(define (domain store-groceries)

    (:requirements :strips :typing :fluents :equality :conditional-effects)

    (:types
        waypoint
        object
        robot
        door
        plane
        context
    )

    (:constants place place_to_store pick pick_to_store - context)

    (:predicates
        (robot_name ?bot - robot)
        (robot_at ?bot - robot ?wp - waypoint)
        (door_at ?door - door ?wp - waypoint)
        (object_at ?obj - object ?wp - waypoint)
        (plane_at ?plane - plane ?wp - waypoint)
        (door_open ?door - door)
        (stored_on ?obj - object ?plane - plane)
        (explored ?plane - plane)
        (on ?obj - object ?plane - plane)
        (holding ?bot - robot ?obj - object)
        (empty_gripper ?bot - robot)
        (groceries_stored)
    )

    (:action move_base
        :parameters (?bot - robot ?from ?to - waypoint)
        :precondition (and
            (robot_at ?bot ?from)
        )
        :effect (and
            (not (robot_at ?bot ?from))
            (robot_at ?bot ?to)
        )
    )

    (:action open_cupboard
        :parameters (?cupboard - door ?bot - robot ?c - waypoint)
        :precondition (and
            (door_at ?cupboard ?c)
            (robot_at ?bot ?c)
        )
        :effect (and
            (door_open ?cupboard)
        )
    )

    (:action perceive_plane
        :parameters (?plane - plane ?bot - robot ?wp - waypoint)
        :precondition (and
            (robot_at ?bot ?wp)
            (plane_at ?plane ?wp)
        )
        :effect (and
            (explored ?plane)
        )
    )

    (:action pickup
        :parameters (?obj - object ?x ?y - plane ?bot - robot ?wp - waypoint ?context - context)
        :precondition (and
            (= ?context pick_to_store)
            (robot_at ?bot ?wp)
            (plane_at ?x ?wp)
            (explored ?x)
            (empty_gripper ?bot)
            (on ?obj ?x)
            (stored_on ?obj ?y)
            (not (= ?x ?y))
        )
        :effect (and
            (not (on ?obj ?x))
            (not (empty_gripper ?bot))
            (holding ?bot ?obj)
        )
    )

    (:action place
        :parameters (?obj - object ?plane - plane ?bot - robot ?wp - waypoint ?context - context)
        :precondition (and
            (= ?context place_to_store)
            (stored_on ?obj ?plane)
            (robot_at ?bot ?wp)
            (plane_at ?plane ?wp)
            (holding ?bot ?obj)
        )
        :effect (and
            (not (holding ?bot ?obj))
            (empty_gripper ?bot)
            (on ?obj ?plane)
            (forall (?obj - object ?plane - plane)
                (when
                    (and
                        (stored_on ?obj ?plane)
                        (on ?obj ?plane)
                    )
                    (groceries_stored)
                )
            )
        )
    )
)
