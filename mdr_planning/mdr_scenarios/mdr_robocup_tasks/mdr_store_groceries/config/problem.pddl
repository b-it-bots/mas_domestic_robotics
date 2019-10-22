(define (problem task)
    (:domain store-groceries)
    (:objects
        cupboard_waypoint table_waypoint home - waypoint
        cup1 phone1 - object
        cup phone - object_class
        hbrs - robot
        cupboard_door - door
        shelf1 shelf2 table - plane
    )
    (:init
        (robot_name hbrs)
        (robot_at hbrs home)

        (door_at cupboard_door cupboard_waypoint)

        (plane_at shelf1 cupboard_waypoint)
        (plane_at shelf2 cupboard_waypoint)
        (plane_at table table_waypoint)

        (stored_on cup shelf1)
        (stored_on phone shelf2)

        (object_category cup1 cup)
        (object_category phone1 phone)

        (on cup1 table)
        (on phone1 table)

        (empty_gripper hbrs)
    )
    (:goal
        (groceries_stored)
    )
)
