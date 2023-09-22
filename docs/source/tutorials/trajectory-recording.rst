Trajectory Recording
====================

Learning Motion Primitives
~~~~~~~~~~~~~~~~~~~~~~~~~~

The process of learning a DMP involves two steps, namely

1. a trajectory demonstration needs to be performed and
2. given the demonstrated trajectory, the weights of the DMP need to be
   learned

Trajectory Demonstration
^^^^^^^^^^^^^^^^^^^^^^^^

To learn a motion primitive, we first need to demonstrate a trajectory
and record it; the
`demonstrated_trajectory_recorder <https://github.com/abhishek098/demonstrated_trajectory_recorder/tree/master>`__
package has been designed for that purpose. For recording a trajectory,
we need to launch the trajectory recording node:

::

   roslaunch demonstrated_trajectory_recorder demo.launch

The command line instructions can then be followed for starting the
recording. After that, the trajectory needs to be demonstrated by moving
the marker rray in front of the robot’s camera (it is clearly important
that the marker array remains visible throughout the demonstration).
Once the demonstration is over, the recording can be stopped; the
trajectory will then be saved to a YAML file with a name specified on
the command line under
``<path-to-demonstrated_trajectory_recorder>/data/file_name.yaml``.

Learning the Weights of the Motion Primitive
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To learn the weights of the motion primitive, we need to use the
functionalities in the
`ros_dmp <https://github.com/abhishek098/ros_dmp/blob/whole_body_motion/src/learn_motion_primitive.py>`__
package - currently on the ``whole_body_motion`` branch. Before
proceeding with the learning step, we need to copy over the demonstrated
trajectory to ``<path-to-ros_dmp>/data/trajectories/old_trajectories/``.
We can now learn the weights by running the learning script (which is
under ``ros_dmp/src``):

::

   python learn_motion_primitive.py

The learning script will ask for the name of the trajectory file (just
the name of the file, not the path); the DMP weights will then be saved
to a YAML file under
``<path-to-ros_dmp>/data/weights/weights_<trajectory_file_name>``.

Using a DMP
^^^^^^^^^^^

To use a DMP in the ``move_arm`` action, the path to the DMP should be
passed as value for the ``dmp_name`` parameter of the action goal. By
passing this parameter, MoveIt! will not be used for motion, but the
trajectory represented by the DMP will be executed instead. As mentioned
at the beginning of this tutorial, DMPs can only be used when moving the
end effector to a specified pose.

**Note 1**: The implementation of the component that takes care of
executing a trajectory represented by a DMP is inside the `move_arm
action <https://github.com/b-it-bots/mas_domestic_robotics/blob/devel/mdr_planning/mdr_actions/mdr_manipulation_actions/mdr_move_arm_action/ros/src/mdr_move_arm_action/dmp.py>`__;
however, as mentioned before, the trajectory represented by the DMP is
for the end effector and not in the joint space. The implementation of
the inverse kinematics solver that actually allows a manipulator to
follow the trajectory can be found in the `mcr_arm_cartesian_control
package <https://github.com/b-it-bots/mas_common_robotics/tree/kinetic/mcr_manipulation/mcr_arm_cartesian_control>`__.

**Note 2**: Due to limitations in the current implementation, we can
only reliably reproduce trajectories with a sideways approach vector;
top-down approach vectors are not guaranteed to work well.

Tuning the DMP Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^

When using the ``move_arm`` action with a learned DMP, users can only
set the :math:`\tau` parameter - ``dmp_tau`` is one of the action goal
parameters - which controls the duration of motion, but also the
accuracy of execution: a value of :math:`1` means that the velocity will
match the velocity of the demonstration, while higher values will
increase the velocity, but may cause oscillations and inaccuracies in
reaching a goal. For the HSR robot, :math:`\tau = 30` has been found to
provide a good balance between accuracy and speed.

During learning, another important parameter is :math:`N`, the number of
basis function used for representing the DMP forcing term. A small
number of basis function generally leads to a crude representation of
the demonstrated trajectory, but reduces the computational cost during
reproduction. On the other hand, a large number of basis functions
increases the accuracy of the representation, but also the complexity of
the reproduction; in addition, a value of :math:`N` that is very high
will lead to learning the noise in the demonstrated trajectory. For
these reasons, the value of :math:`N` is best determined experimentally
and may need to vary from trajectory to trajectory, such that values
:math:`N = 300` or :math:`N = 500` may be good initial guesses.