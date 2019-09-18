(define (domain door_open)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

    (:types
        waypoint
        robot
    )

    (:predicates
        (robot_name ?bot - robot)
        (robot_at ?bot - robot ?wp - waypoint)
    )

    (:durative-action move_base
        :parameters (?bot - robot ?from ?to - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?bot ?from))
        )
        :effect (and
            (at start (not (robot_at ?bot ?from)))
            (at end (robot_at ?bot ?to))
        )
    )
)

