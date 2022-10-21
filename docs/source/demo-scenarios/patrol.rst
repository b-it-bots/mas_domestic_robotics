Patrolling Demo
===============

State machine
-------------

* **State machine id:** mdr_demo_patrol  

* **States:** [GO_TO]

* **Outcomes:** [FAILED]

States
------
1. GO_TO  

  *  module: ``mdr_navigation_behaviours.move_base``
  *  class name: ``MoveBase``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> GO_TO
       - failed -> GO_TO
       - failed_after_retrying -> FAILED

  *  **Arguments:** (name):(value)
        
       - destination_locations: [kitchen, living_room, table, shelf, table]
       - number_of_retries:3

