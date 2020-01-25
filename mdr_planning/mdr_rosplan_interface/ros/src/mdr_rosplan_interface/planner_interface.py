import rospy
from std_srvs.srv import Empty
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_dispatch_msgs.srv as plan_dispatch_srvs
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface

class PlannerInterface(object):
    '''An interface providing functionalities for interacting with a planner.

    The interface expects four planner services to be exposed, whose names should be
    made available to the parameter server as follows:
    * /planner/problem_generation_srv: Name of a service that generates a planner problem
    * /planner/planner_srv: Name of a service that invokes a planner
    * /planner/plan_parsing_srv: Name of a service that parses a generated plan
    * /planner/plan_dispatch_srv: Name of a service that invokes a plan dispatcher

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        self.problem_generation_proxy = None
        self.planner_proxy = None
        self.plan_parsing_proxy = None
        self.plan_dispatch_proxy = None
        self.action_dispatch_topic = rospy.get_param('/planner/action_dispatch_topic',
                                                     '/kcl_rosplan/action_dispatch')
        self.action_feedback_topic = rospy.get_param('/planner/action_feedback_topic',
                                                     '/kcl_rosplan/action_feedback')
        self.kb_interface = DomesticKBInterface()

        problem_generation_srv = rospy.get_param('/planner/problem_generation_srv',
                                                 '/rosplan_problem_interface/problem_generation_server')
        try:
            rospy.wait_for_service(problem_generation_srv, 5.)
            self.problem_generation_proxy = rospy.ServiceProxy(problem_generation_srv, Empty)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logerr('The service %s does not appear to exist.', problem_generation_srv)

        planner_srv = rospy.get_param('/planner/planner_srv',
                                      '/rosplan_planner_interface/planning_server')
        try:
            rospy.wait_for_service(planner_srv, 5.)
            self.planner_proxy = rospy.ServiceProxy(planner_srv, Empty)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logerr('The service %s does not appear to exist.', planner_srv)

        plan_parsing_srv = rospy.get_param('/planner/plan_parsing_srv',
                                           '/rosplan_parsing_interface/parse_plan')
        try:
            rospy.wait_for_service(plan_parsing_srv, 5.)
            self.plan_parsing_proxy = rospy.ServiceProxy(plan_parsing_srv, Empty)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logerr('The service %s does not appear to exist.', plan_parsing_srv)

        plan_dispatch_srv = rospy.get_param('/planner/plan_dispatch_srv',
                                            '/rosplan_plan_dispatcher/dispatch_plan')
        try:
            rospy.wait_for_service(plan_dispatch_srv, 5.)
            self.plan_dispatch_proxy = rospy.ServiceProxy(plan_dispatch_srv,
                                                          plan_dispatch_srvs.DispatchService)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logerr('The service %s does not appear to exist.', plan_dispatch_srv)

        self.current_action = ''
        self.executing = False
        self.action_dispatch_sub = rospy.Subscriber(self.action_dispatch_topic,
                                                    plan_dispatch_msgs.ActionDispatch,
                                                    self.get_dispatched_action)
        self.action_feedback_sub = rospy.Subscriber(self.action_feedback_topic,
                                                    plan_dispatch_msgs.ActionFeedback,
                                                    self.get_action_feedback)

    def add_plan_goals(self, goals):
        '''Registers the ground predicates in the given list as planning goals.

        Keyword arguments:
        @param goal_list -- a list in which each entry is a tuple of the form
                            (predicate, [(variable, value), ...]), where
                            "predicate" is the predicate name and the
                            (variable, value) tuples are the variable values

        '''
        self.kb_interface.insert_goals(goals)

    def remove_plan_goals(self, goals):
        '''Removes the ground predicates in the given list from the list of planning goals.

        Keyword arguments:
        @param goal_list -- a list in which each entry is a tuple of the form
                            (predicate, [(variable, value), ...]), where
                            "predicate" is the predicate name and the
                            (variable, value) tuples are the variable values

        '''
        self.kb_interface.remove_goals(goals)

    def plan(self):
        '''Generates a problem file from the current state of the knowledge base
        and invokes the planner.
        '''
        rospy.loginfo('[planner_interface] Generating a problem file')
        try:
            self.problem_generation_proxy()
        except rospy.ServiceException as exc:
            rospy.logerr('An error occured while generating a problem file: ' + str(exc))
            return False

        rospy.loginfo('[planner_interface] Invoking the planner')
        try:
            self.planner_proxy()
        except rospy.ServiceException as exc:
            rospy.logerr('An error occured while planning: ' + str(exc))
            return False
        return True

    def start_plan_dispatch(self):
        '''Parses a previously generated plan and starts dispatching the plan.
        '''
        rospy.loginfo('[planner_interface] Parsing the plan')
        try:
            self.plan_parsing_proxy()
        except rospy.ServiceException as exc:
            rospy.logerr('An error occured while parsing the plan: ' + str(exc))
            return False

        rospy.loginfo('[planner_interface] Invoking the plan dispatcher')
        try:
            self.plan_dispatch_proxy()
        except rospy.ServiceException as exc:
            rospy.logerr('An error occured while dispatching the plan: ' + str(exc))
            return False

        self.executing = True
        return True

    def get_current_action(self):
        return self.current_action

    def get_dispatched_action(self, msg):
        self.current_action = msg.name

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.current_action:
            self.executing = msg.status != 'action achieved'
