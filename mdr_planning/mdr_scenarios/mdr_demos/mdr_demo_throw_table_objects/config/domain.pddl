(define (domain throw-table-objects)

    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

    (:types
        waypoint
        object
        category
        robot
        plane
    )

    (:predicates
        (robot_name ?bot - robot)
        (robot_at ?bot - robot ?wp - waypoint)
        (object_at ?obj - object ?wp - waypoint)
        (plane_at ?plane - plane ?wp - waypoint)
        (unexplored ?plane - plane)
        (explored ?plane - plane)
        (on ?obj - object ?plane - plane)
        (in ?obj - object ?obj - object)
        (holding ?bot - robot ?obj - object)
        (empty_gripper ?bot - robot)
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

    (:durative-action perceive_plane
        :parameters (?plane - plane ?bot - robot ?wp - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?bot ?wp))
            (at start (plane_at ?plane ?wp))
            (at start (unexplored ?plane))
        )
        :effect (and
            (at start (not (unexplored ?plane)))
            (at end (explored ?plane))
        )
    )

    (:durative-action pickup
        :parameters (?obj - object ?plane - plane ?bot - robot ?wp - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?bot ?wp))
            (at start (plane_at ?plane ?wp))
            (at start (explored ?plane))
            (at start (on ?obj ?plane))
            (at start (empty_gripper ?bot))
        )
        :effect (and
            (at start (not (on ?obj ?plane)))
            (at start (not (empty_gripper ?bot)))
            (at end (holding ?bot ?obj))
        )
    )

    (:durative-action throw
        :parameters (?obj - object ?dest_obj - object ?bot - robot ?wp - waypoint)
        :duration ( = ?duration 10)
        :condition (and
            (at start (robot_at ?bot ?wp))
            (at start (object_at ?dest_obj ?wp))
            (at start (holding ?bot ?obj))
        )
        :effect (and
            (at start (not (holding ?bot ?obj)))
            (at start (empty_gripper ?bot))
            (at end (in ?obj ?dest_obj))
        )
    )
)
