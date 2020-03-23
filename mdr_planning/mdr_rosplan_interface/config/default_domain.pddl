(define (domain default-domestic-domain)

    (:requirements :strips :typing :equality)

    (:types
        Thing
        Waypoint
        Object - Thing
        Furniture - Object
        Category
        Robot
        Door
        Plane
        Person
        NamedPose
        GraspingStrategy
        Context
    )

    (:constants pick_from_plane pick_from_container place_on_plane place_in_container - Context)

    (:predicates
        (robotName ?Robot - Robot)
        (objectCategory ?Object0 - Object ?Object1 - Object)
        (robotAt ?Robot - Robot ?Waypoint - Waypoint)
        (doorAt ?Door - Door ?Waypoint - Waypoint)
        (objectAt ?Object - Object ?Waypoint - Waypoint)
        (furnitureAt ?Furniture - Furniture ?Waypoint - Waypoint)
        (planeAt ?Plane - Plane ?Waypoint - Waypoint)
        (personAt ?Person - Person ?Waypoint - Waypoint)
        (doorOpen ?Door - Door)
        (belongsTo ?Plane - Plane ?Object - Object)
        (unexplored ?Plane - Plane)
        (explored ?Plane - Plane)
        (on ?Object - Object ?Plane - Plane)
        (in ?Object0 - Object ?Object1 - Object)
        (in ?Object - Object ?Furniture - Furniture)
        (holding ?Robot - Robot ?Object - Object)
        (holding ?Person - Person ?Object - Object)
        (emptyGripper ?Robot - Robot)
        (known ?Person - Person)
        (unknown ?Person - Person)

        (canPlaceOn ?Object - Object ?Plane - Plane)
        (defaultStoringLocation ?Object - Object ?Furniture - Furniture)
        (likelyLocation ?Object - Object ?Furniture - Furniture)
        (locatedAt ?Object - Object ?Location - Location)
        (hasDoor ?Furniture - Furniture)
        (above ?Object0 - Object ?Object1 - Object)
        (below ?Object0 - Object ?Object1 - Object)
        (onTopOf ?Object0 - Object ?Object1 - Object)
        (inside ?Object0 - Object ?Object1 - Object)
        (toTheLeftOf ?Object0 - Object ?Object1 - Object)
        (toTheRightOf ?Object0 - Object ?Object1 - Object)
        (isAtNamedPose ?Thing - Thing ?NamedPose - NamedPose)
        (isAtLocation ?NamedPose - NamedPose ?Location - Location)
        (preferredGraspingStrategy ?Object - Object ?GraspingStrategy - GraspingStrategy)
    )

    (:action MoveBase
        :parameters (?Robot - Robot ?Waypoint0 ?Waypoint1 - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint0)
        )
        :effect (and
            (not (robotAt ?Robot ?Waypoint0))
            (robotAt ?Robot ?Waypoint1)
        )
    )

    (:action Open
        :parameters (?Door - Door ?Robot - Robot ?Waypoint - Waypoint)
        :precondition (and
            (doorAt ?Door ?Waypoint)
            (robotAt ?Robot ?Waypoint)
        )
        :effect (and
            (doorOpen ?Door)
        )
    )

    (:action PerceivePlane
        :parameters (?Plane - Plane ?Robot - Robot ?Waypoint - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint)
            (planeAt ?Plane ?Waypoint)
            (unexplored ?Plane)
        )
        :effect (and
            (not (unexplored ?Plane))
            (explored ?Plane)
        )
    )

    (:action Pick
        :parameters (?Object - Object ?Plane - Plane ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context pick_from_plane)
            (robotAt ?Robot ?Waypoint)
            (planeAt ?Plane ?Waypoint)
            (explored ?Plane)
            (on ?Object ?Plane)
            (emptyGripper ?Robot)
        )
        :effect (and
            (not (on ?Object ?Plane))
            (not (emptyGripper ?Robot))
            (holding ?Robot ?Object)
        )
    )

    (:action Pick
        :parameters (?Object - Object ?Furniture - Furniture ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context pick_from_container)
            (robotAt ?Robot ?Waypoint)
            (furnitureAt ?Furniture ?Waypoint)
            (in ?Object ?Furniture)
            (emptyGripper ?Robot)
        )
        :effect (and
            (not (in ?Object ?Furniture))
            (not (emptyGripper ?Robot))
            (holding ?Robot ?Object)
        )
    )

    (:action Place
        :parameters (?Object - Object ?Plane - Plane ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context place_on_plane)
            (robotAt ?Robot ?Waypoint)
            (planeAt ?Plane ?Waypoint)
            (holding ?Robot ?Object)
        )
        :effect (and
            (not (holding ?Robot ?Object))
            (emptyGripper ?Robot)
            (on ?Object ?Plane)
        )
    )

    (:action Place
        :parameters (?Object - Object ?Furniture - Furniture ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context place_in_container)
            (robotAt ?Robot ?Waypoint)
            (furnitureAt ?Furniture ?Waypoint)
            (holding ?Robot ?Object)
        )
        :effect (and
            (not (holding ?Robot ?Object))
            (emptyGripper ?Robot)
            (in ?Object ?Furniture)
        )
    )

    (:action Throw
        :parameters (?Object0 ?Object1 - Object ?Robot - Robot ?Waypoint - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint)
            (objectAt ?Object1 ?Waypoint)
            (holding ?Robot ?Object0)
        )
        :effect (and
            (not (holding ?Robot ?Object0))
            (emptyGripper ?Robot)
            (in ?Object0 ?Object1)
        )
    )

    (:action HandOver
        :parameters (?Object - Object ?Robot - Robot ?Person - Person ?Waypoint - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint)
            (personAt ?Person ?Waypoint)
            (holding ?Robot ?Object)
        )
        :effect (and
            (not (holding ?Robot ?Object))
            (holding ?Person ?Object)
        )
    )
)
