^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_simple_place
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2017-10-29)
------------------
* Update permissions of non-executable files
  Files in src are now not executable
* Contributors: Argentina Ortega Sainz

1.1.1 (2017-09-20)
------------------
* Change maintainer tags to MAS Robotics
* Update changelog for mdr_planning
* Contributors: Argentina Ortega Sainz

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------
* Merge branch 'refactor/mdr_msgs' into 'indigo'
  Create a new mdr_msgs metapackage and move all messages/services there
  See merge request !27
* Renamed 'mdr_behaviors_msgs' to 'mdr_behavior_msgs'
* Merge branch 'refactor/mdr_planning' into 'indigo'
  Move actions, behaviors, and scenarios to a new mdr_planning metapackage
  See merge request !25
* Moved actions, behaviours, and scenarios to mdr_planning
* Contributors: Alex Mitrevski

1.0.0 (2017-04-11 10:44:12 +0200)
---------------------------------
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
* Cleanup dependencies
* Added simple place component
* Contributors: Frederik Hegger, Sven Schneider, Alex Moriarty
