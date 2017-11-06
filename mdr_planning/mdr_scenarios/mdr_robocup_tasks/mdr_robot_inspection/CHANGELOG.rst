^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_robot_inspection_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2017-11-04)
------------------
* Added functionality for resetting the internal state of the 'WaitForCmd' and 'WaitForQR' states
* Add amcl pose mean to SM initialization
  The script gets the pose from the script server and sets the value
  as the pose mean parameter in amcl.
* Added actions to launch file
* Added the existing states from 'action_states' to the scenario SM
* Added an introduce self state, wait for command, acknowledge,
  enter, move base, move tray, and wait for QR states
* Move mdr_robot_inspection to mdr_robocup_tasks
* Contributors: Alex Mitrevski, Argentina Ortega Sainz

1.1.2 (2017-10-29)
------------------

1.1.1 (2017-09-20)
------------------
* Change maintainer tags to MAS Robotics
* Update changelog for mdr_planning
* Contributors: Argentina Ortega Sainz

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------
* Merge branch 'refactor/mdr_planning' into 'indigo'
  Move actions, behaviors, and scenarios to a new mdr_planning metapackage
  See merge request !25
* Refactored mdr_actions
* Moved actions, behaviours, and scenarios to mdr_planning
* Contributors: Alex Mitrevski

1.0.0 (2017-04-11 10:44:12 +0200)
---------------------------------
* Merge branch 'indigo' into 'perceive_table'
  # Conflicts:
  #   mdr_behaviors/mdr_actions/CMakeLists.txt
  #   mdr_behaviors/mdr_actions/ros/src/mdr_actions/action_states.py
* Merge branch 'inspection-test' into 'indigo'
  [scenarios] actions and SM for robot inspection test
  See merge request !15
* [scenarios] actions and SM for robot inspection test
* Contributors: Minh Nguyen, Santosh Thoduka
