(define (problem hand-over-problem)
    (:domain default-domestic-domain)
    (:objects
        robot1 - robot
        wp0 - waypoint
    )
    (:init
        (robot_name robot1)
        (robot_at robot1 wp0)
        (empty_gripper robot1)
    )
    (:goal
        (robot_at robot1 wp0)
    )
)
