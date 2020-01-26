(define (problem task)
    (:domain store-groceries)
    (:objects
        cupboard_waypoint table_waypoint home - waypoint
        cup1 phone1 cup2 phone2 bottle1 - object
        cup phone bottle - object_class
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

        (object_category cup1 cup)
        (object_category phone1 phone)
        (object_category bottle1 bottle)
        (object_category cup2 cup)
        (object_category phone2 phone)

        (stored_on cup shelf1)
        (stored_on bottle shelf1)
        (stored_on phone shelf2)

        (on cup1 table)
        (on phone1 table)
        (on bottle1 table)
        (on cup2 shelf1)
        (on phone2 shelf2)

        (empty_gripper hbrs)
    )
    (:goal (and
        (on cup1 shelf1)
        (on phone1 shelf2)
        (on bottle1 shelf1)
    ))
)
