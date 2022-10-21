Simple Object Pick and Place Demo
=================================

State machine
-------------

* **State machine id:** mdr_demo_simple_pick_and_place  

* **States:** [GO_TO_TABLE, PERCEIVE_TABLE, PICK_OBJECT, PLACE_OBJECT]

* **Outcomes:** [FAILED]

States
------
1. GO_TO_TABLE  

  *  module: ``mdr_navigation_behaviours.move_base``
  *  class name: ``MoveBase``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> PERCEIVE_TABLE
       - failed -> GO_TO_TABLE
       - failed_after_retrying -> FAILED

  *  **Arguments:** (name):(value)
        
       - destination_locations: [table]
       - number_of_retries:3

2. PERCEIVE_TABLE  

  *  module: ``mdr_perception_behaviours.perceive_planes``
  *  class name: ``PerceivePlanes``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> PICK_OBJECT
       - failed -> PERCEIVE_TABLE
       - failed_after_retrying -> FAILED

  *  **Arguments:** (name):(value)
        
       - plane_prefix: table
       - number_of_retries:3

3. PICK_OBJECT  

  *  module: ``mdr_manipulation_behaviours.pick_closest_from_surface``
  *  class name: ``PickClosestFromSurface``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> PLACE_OBJECT
       - failed -> PICK_OBJECT
       - find_objects_before_picking -> PERCEIVE_TABLE

  *  **Arguments:** (name):(value)
        
       - picking_surface_prefix: table
       - number_of_retries:3

4. PLACE_OBJECT  

  *  module: ``mdr_manipulation_behaviours.place``
  *  class name: ``Place``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> PERCEIVE_TABLE
       - failed -> PLACE_OBJECT
       - failed_after_retrying -> FAILED

  *  **Arguments:** (name):(value)
        
       - placing_surface_prefix: table
       - number_of_retries:3