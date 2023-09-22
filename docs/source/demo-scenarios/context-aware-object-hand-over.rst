Context-Aware Object Hand Over Demo
===================================

State machine
-------------

* **State machine id:** mdr_demo_context_aware_hand_over  

* **States:** [DETECT_PERSON, IDENTIFY_POSTURE, HAND_OVER]

* **Outcomes:** [DONE, FAILED]

States
------
1. DETECT_PERSON  

  *  module: ``mdr_demo_context_aware_hand_over.scenario_states.detect_person``
  *  class name: ``DetectPerson``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> IDENTIFY_POSTURE
       - failed -> DETECT_PERSON
       - failed_after_retrying -> FAILED

  *  **Arguments:** (name):(value)
        
       - number_of_retries:3


2. IDENTIFY_POSTURE  

  *  module: ``mdr_demo_context_aware_hand_over.scenario_states.identify_posture``
  *  class name: ``IdentifyPosture``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> HAND_OVER
       - failed -> DETECT_PERSON

  *  **Arguments:** (name):(value)
        
       - posture_height_width_ratio_ranges:
           - standing: [1.65, 5.0]
           - seated: [0.6, 1.65]
           - lying: [0.0, 0.6] 

3. HAND_OVER  

  *  module: ``mdr_demo_context_aware_hand_over.scenario_states.hand_over``
  *  class name: ``HandOver``  
  *  **Transitions:** (name)->(transition to the state)

       - succeeded -> DONE
       - failed -> HAND_OVER
       - failed_after_retrying -> FAILED

  *  **Arguments:** (name):(value)
        
       - number_of_retries:3
       - obstacle_present: False
       - context_aware: True
       - release_detection: True