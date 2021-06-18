#!/usr/bin/python
from threading import Thread

import rospy
import moveit_commander

from mdr_manipulation_msgs.srv import UpdatePlanningScene, UpdatePlanningSceneRequest, UpdatePlanningSceneResponse

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmGoal, MoveArmResult
from mdr_move_arm_action.dmp import DMPExecutor

class MoveArmSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 arm_name='arm',
                 whole_body_name='',
                 update_planning_scene_srv_name='/move_arm_action/update_planning_scene',
                 workspace_bounds=[-10., -10., 10., 10.],
                 max_recovery_attempts=1):
        super(MoveArmSM, self).__init__('MoveArm', [], max_recovery_attempts)
        self.timeout = timeout
        self.arm_name = arm_name
        self.whole_body_name = whole_body_name
        self.update_planning_scene_srv_name = update_planning_scene_srv_name
        self.arm = None
        self.whole_body = None
        self.end_effector = None
        self.planning_scene = None
        self.update_planning_scene_server = None

    def init(self):
        try:
            rospy.loginfo('[move_arm] Initialising group %s', self.arm_name)
            self.arm = moveit_commander.MoveGroupCommander(self.arm_name)
            rospy.loginfo('[move_arm] Group %s initialised', self.arm_name)
        except Exception as exc:
            rospy.logerr('[move_arm] %s could not be initialised: %s',
                         self.arm_name, str(exc))
            return FTSMTransitions.INIT_FAILED

        if self.whole_body_name:
            try:
                rospy.loginfo('[move_arm] Initialising group %s', self.whole_body_name)
                self.whole_body = moveit_commander.MoveGroupCommander(self.whole_body_name)
                self.whole_body.allow_replanning(True)
                self.whole_body.set_planning_time(5)
                self.whole_body.set_workspace(workspace_bounds)
                self.end_effector = self.whole_body.get_end_effector_link()
                rospy.loginfo('[move_arm] Group %s initialised', self.whole_body_name)
            except Exception as exc:
                rospy.logerr('[move_arm] %s could not be initialised: %s',
                             self.whole_body_name, str(exc))
                return FTSMTransitions.INIT_FAILED
        else:
            rospy.loginfo('[move_arm] whole_body_name not specified; not initialising whole body group')

        rospy.loginfo('[move_arm] Creating MoveIt! planning scene')
        self.planning_scene = moveit_commander.PlanningSceneInterface()

        rospy.loginfo('[move_arm] Exposing server for planning scene updates as %s', self.update_planning_scene_srv_name)
        self.update_planning_scene_server = rospy.Service(self.update_planning_scene_srv_name,
                                                          UpdatePlanningScene,
                                                          self.update_planning_scene_handler)
        rospy.loginfo('[move_arm] Move arm action initialised')

        return FTSMTransitions.INITIALISED

    def running(self):
        self.arm.clear_pose_targets()

        if self.whole_body:
            self.whole_body.clear_pose_targets()
            self.planning_scene.remove_attached_object(self.end_effector)
        rospy.sleep(1)

        success = False
        if self.goal.goal_type == MoveArmGoal.NAMED_TARGET:
            self.arm.set_named_target(self.goal.named_target)
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')
            success = self.arm.go(wait=True)
        elif self.goal.goal_type == MoveArmGoal.END_EFFECTOR_POSE:
            goal = self.goal.end_effector_pose
            dmp_name = self.goal.dmp_name
            tau = self.goal.dmp_tau
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')

            # we use a dynamic motion primitive for moving the arm if one is specified;
            # otherwise, we just use moveit for planning a trajectory and moving the arm
            if dmp_name:
                rospy.loginfo('[move_arm] Using a DMP for arm motion')
                dmp_traj_executor = DMPExecutor(dmp_name, tau)
                dmp_execution_thread = Thread(target=dmp_traj_executor.move_to, args=(goal,))
                dmp_execution_thread.start()
                while not dmp_traj_executor.motion_completed and \
                      not self.preempted:
                    rospy.sleep(0.05)

                if self.preempted:
                    dmp_traj_executor.motion_cancelled = True
                    self.preempted = False

                    rospy.loginfo('[move_arm] Cancelled arm motion')
                    self.result = self.set_result(False)
                    return FTSMTransitions.DONE
            else:
                if self.whole_body:
                    rospy.loginfo('[move_arm] Planning whole body motion...')
                    self.whole_body.set_pose_reference_frame(goal.header.frame_id)
                    self.whole_body.set_pose_target(goal.pose)
                    success = self.whole_body.go(wait=True)
                else:
                    rospy.loginfo('[move_arm] Planning arm motion...')
                    self.arm.set_pose_reference_frame(goal.header.frame_id)
                    self.arm.set_pose_target(goal.pose)
                    success = self.arm.go(wait=True)
        elif self.goal.goal_type == MoveArmGoal.JOINT_VALUES:
            joint_values = self.goal.joint_values
            self.arm.set_joint_value_target(joint_values)
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')
            success = self.arm.go(wait=True)
        else:
            rospy.logerr('[move_arm] Invalid target specified; ignoring request')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        if not success:
            rospy.logerr('[move_arm] Arm motion unsuccessful')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        rospy.loginfo('[move_arm] Arm motion successful')
        self.result = self.set_result(True)
        return FTSMTransitions.DONE

    def recovering(self):
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = MoveArmResult()
        result.success = success
        return result

    def update_planning_scene_handler(self, request):
        rospy.loginfo('[move_arm] Received a planning scene update request to %s %d objects', request.operation,
                                                                                              len(request.objects.objects))
        response = UpdatePlanningSceneResponse(success=False)
        if request.operation == UpdatePlanningSceneRequest.ADD:
            try:
                for obj in request.objects.objects:
                    self.planning_scene.add_box(obj.name, obj.pose,
                                                (obj.dimensions.vector.x,
                                                 obj.dimensions.vector.y,
                                                 obj.dimensions.vector.z))
                response.success = True
            except Exception as exc:
                rospy.logerr('[move_arm/update_planning_scene_handler] Error while adding objects')
                rospy.logerr(str(exc))
        elif request.operation == UpdatePlanningSceneRequest.REMOVE:
            try:
                for obj in request.objects.objects:
                    self.planning_scene.remove_world_object(obj.name)
                response.success = True
            except Exception as exc:
                rospy.logerr('[move_arm/update_planning_scene_handler] Error while removing objects')
                rospy.logerr(str(exc))
        else:
            rospy.logerr('[move_arm/update_planning_scene_handler] Received unknown operation %s', request.operation)
        return response
