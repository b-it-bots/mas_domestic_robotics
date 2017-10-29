^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_emergency_stop_mapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2017-10-29)
------------------

1.1.1 (2017-09-20)
------------------
* Change maintainer tags to MAS Robotics
* Generate changelogs for mdr_monitoring
* Contributors: Argentina Ortega Sainz

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------

1.0.0 (2017-04-11)
------------------
* Merge branch 'catkin_lint' into 'indigo'
  [CATKIN_LINT] fix issues reported by catkin lint
  - Adds missing CATKIN_DEPENDS to catkin_package()
  - Adds missing install()
  - mdr_lab_demo is probably broken: It depends on a package which doesn't exist, replaced with the closest thing (or maybe old package was just renamed)
  See merge request !6
* [CATKIN_LINT] fix issues reported by catkin_lint
* Merge branch 'indigo' into 'indigo'
  Catkin Changes & Package updates for Indigo
  - Updates setup.py for newer version of catkin
  - Updates repository.debs for indigo versions
  - Updates repository.rosinstall to get indigo version from ipa320, our forks aren't up to date (yet)
  - Updates EmergencyStopState includes to new location:
  In cob_relay 0.5.6 the EmergencyStopState was moved
  http://docs.ros.org/indigo/changelogs/cob_relayboard/changelog.html
  See merge request !4
* EmergencyStopState has moved to cob_msgs
  - In 0.5.6 the EmergencyStopeState was moved:
  http://docs.ros.org/indigo/changelogs/cob_relayboard/changelog.html
* Update dependencies of emergency stop mapper
* Remove not existing include dir
* Fix emergency stop mapper
* Remap to current topic of the arm's external emergency stop
* Cleanup emergency stop mapper
* Add launch file for emergency stop mapper
* Add translation from cob representation of emergency stop status to std_msgs representation
* Contributors: Frederik Hegger, Sven Schneider, Alex Moriarty
