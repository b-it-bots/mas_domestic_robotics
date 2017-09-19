^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_common_states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'refactor/mdr_msgs' into 'indigo'
  Create a new mdr_msgs metapackage and move all messages/services there
  See merge request !27
* Renamed 'mdr_behaviors_msgs' to 'mdr_behavior_msgs'
* rm unused CleanTable srv and clean_table state
* rm unused GetWrench service
* Merge branch 'refactor/mdr_lab_demo' into 'indigo'
  Remove mdr_common and mdr_lab_demo
  See merge request !26
* rm mdr_common
* Merge branch 'refactor/mdr_planning' into 'indigo'
  Move actions, behaviors, and scenarios to a new mdr_planning metapackage
  See merge request !25
* Moved actions, behaviours, and scenarios to mdr_planning
* Contributors: Alex Mitrevski

1.0.0 (2017-04-11 10:44:12 +0200)
---------------------------------
* Merge Perceive Table Skill to indigo
  See merge request !17
* Merge branch 'indigo' into 'perceive_table'
  # Conflicts:
  #   mdr_behaviors/mdr_actions/CMakeLists.txt
  #   mdr_behaviors/mdr_actions/ros/src/mdr_actions/action_states.py
* Add perceive_table skill from minh/go2017 branch
* Merge branch 'inspection-test' into 'indigo'
  [scenarios] actions and SM for robot inspection test
  See merge request !15
* [scenarios] actions and SM for robot inspection test
* [common states] remove tabs in empty lines and at line ends
* [common_states] remove usage of non-existing mcr_navigation_msgs
  and comment code retrieving the base pose. This needs to be modified to use TF
* Merge branch 'indigo' into 'indigo'
  Catkin Changes & Package updates for Indigo
  - Updates setup.py for newer version of catkin
  - Updates repository.debs for indigo versions
  - Updates repository.rosinstall to get indigo version from ipa320, our forks aren't up to date (yet)
  - Updates EmergencyStopState includes to new location:
  In cob_relay 0.5.6 the EmergencyStopState was moved
  http://docs.ros.org/indigo/changelogs/cob_relayboard/changelog.html
  See merge request !4
* Fix Python Packages for latest Catkin
  see: https://github.com/ros/catkin/issues/756
* Move torso to HOME before moving the arm to folded
* Use Pickup service insteado of Grasp service
* Use additional varibale for the service name
* Use the body detection topic instead of the service
* Use new event-based interface of the body detection component
* Add variable for the service name
* Add try/catch for wait_for_message
* Use std_msg/String msg instead of custom msg
* Adapt to changed structure of GetObjectList service
* Adapt to new object list name
* Correct overtray and tray position names
* Fix import for set light color function
* Adjust topics for haptic guidance
* Use latched publisher
* Add missing parenthese
* Use latch option
* Remove not existing arm handles
* Listen to the event_out topic of the speech synthesis to provide blocking and unblocking say functionality
* Add missing import of speech states
* Adjust speech service names and use a variable for the service name
* Replace simple script server recover of the arm with a service call
* Add missing import of the moveit commander
* Remove old roslib import and manifest loading
* Remove unused service client and publisher
* Remove commented lines
* Comment lines which change the joint stiffness of the arm
* Replace script server move "arm" call with the respective moveit commands
* Fixed pick and place scenario
* Adapted service names
* Add pickup and place state
* Adjust service names, topic names and service/message types
* Remove brsu_prefix
* Add setup.py file
* Change authors
* Add common states package
* Contributors: Frederik Hegger, Hans Wurst, Minh Nguyen, Santosh Thoduka, Sven Schneider, Jose Sanchez, Matthias Füller, Alex Moriarty
