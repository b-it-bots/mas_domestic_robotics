(define (problem pick-and-place-problem)
    (:domain pick-and-place)
    (:objects
        robot1 - robot
        wp0 - waypoint
    )
    (:init
        (robot_at robot1 wp0)
        (empty_gripper robot1)
    )
    (:goal
        (robot_at robot1 wp0)
    )
)
