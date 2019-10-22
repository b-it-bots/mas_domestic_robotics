(define (domain store-groceries)

    (:requirements :strips :typing :fluents :equality :conditional-effects)

    (:types
        waypoint
        object
        object_class
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
        (object_category ?obj - object ?class - object_class)
        (stored_on ?class - object_class ?plane - plane)
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
        :parameters (?obj - object ?class - object_class ?x ?y - plane ?bot - robot ?wp - waypoint ?context - context)
        :precondition (and
            (= ?context pick_to_store)
            (empty_gripper ?bot)
            (robot_at ?bot ?wp)
            (plane_at ?x ?wp)
            (explored ?x)
            (on ?obj ?x)
            (object_category ?obj ?class)
            (stored_on ?class ?y)
            (not (= ?x ?y))
        )
        :effect (and
            (not (on ?obj ?x))
            (not (empty_gripper ?bot))
            (holding ?bot ?obj)
        )
    )

    (:action place
        :parameters (?obj - object ?class - object_class ?plane - plane ?bot - robot ?wp - waypoint ?context - context)
        :precondition (and
            (= ?context place_to_store)
            (robot_at ?bot ?wp)
            (plane_at ?plane ?wp)
            (holding ?bot ?obj)
            (object_category ?obj ?class)
            (stored_on ?class ?plane)
        )
        :effect (and
            (not (holding ?bot ?obj))
            (empty_gripper ?bot)
            (on ?obj ?plane)
            (forall (?o - object ?c - object_class ?p - plane)
                (when
                    (and
                        (object_category ?o ?c)
                        (stored_on ?c ?p)
                        (on ?o ?p)
                    )
                    (groceries_stored)
                )
            )
        )
    )
)
