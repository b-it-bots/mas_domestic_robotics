Arm Motion Based on Dynamic Motion Primitives
=============================================

Dynamic Motion Primitives (DMPs)
--------------------------------

For the purpose of easier setup and portability, our robots have
traditionally used `MoveIt! <https://moveit.ros.org>`__ as a
manipulation framework. As described above, our ``move_arm`` action uses
MoveIt! as well, but not exclusively, such that we additionally make use
of dynamic motion primitives [1].

MoveIt! generally uses randomised planners for finding motion
trajectories for a robot manipulator. This offers flexibility in the
motion planning process; however, randomised planners often result in
trajectories that are unpredictable or suboptimal (e.g. with respect to
the distance travelled). Dynamic motion primitives on the other hand
encode trajectories that a manipulator will then try to reproduce during
its motion; one benefit of this is that resulting trajectories are more
predictable and often more natural.

DMP Primer
~~~~~~~~~~

In the DMP framework [1], trajectories are represented by a second-order
differential equation of the form

.. math::

        \tau \ddot{\mathbf{y}} = \alpha(\beta(\mathbf{g} - \mathbf{y}) - \dot{\mathbf{y}}) + \mathbf{f}


In this equation, :math:`\mathbf{y}` is the state of the system (usually
the 3D Cartesian pose of the robot), :math:`\mathbf{g}` is the desired
end effector pose, :math:`\tau` is a parameter that adjusts the
trajectory duration, while :math:`\alpha` and :math:`\beta` are positive
constants (please consult `these very nice lecture
notes <http://tutorial.math.lamar.edu/Classes/DE/Vibrations.aspx>`__ if
you need a refresher on second-order differential equations and
spring-mass-damper systems in particular).

A common use for DMPs is to encode trajectories that have been
demonstrated to a robot, such that the goal in this case is finding a
representation for :math:`\mathbf{f}` that will represent the
demonstrated trajectory as closely as possible; the forcing term should
however eventually vanish so that the trajectory can converge to the
desired goal. In [1], which is what our implementation is based on, the
forcing term is represented as

.. math::

        f_j(t) = \frac{\sum_{i=1}^{N}\Psi_{i,j}(t)w_{i,j}}{\sum_{i=1}^{N}\Psi_{i,j}(t)}


where the :math:`\Psi_{i,j}` are exponential basis functions and the
:math:`w_{i,j}` are weighting terms for the individual basis functions.

We demonstrate trajectories by moving a marker array and recording the
observations of the marker using a robot’s camera (a video illustrating
the demonstration process can be found at
https://www.youtube.com/watch?v=jEtlm96KAbA); given a demonstration of
the trajectory, we can learn the trajectory weights and then use them
for execution. The learning process is described next.

[1] A. J. Ijspeert, J. Nakanishi, H. Hoffmann, P. Pastor, and S. Schaal,
“Dynamical Movement Primitives: Learning Attractor Models for Motor
Behaviors,” Neural Computation, vol. 25, no. 2, pp. 328-373, 2013.
Available:
https://homes.cs.washington.edu/~todorov/courses/amath579/reading/DynamicPrimitives.pdf