^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_tactile_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2017-11-04)
------------------

1.1.2 (2017-10-29)
------------------
* Update permissions of non-executable files
  Files in src are now not executable
* Contributors: Argentina Ortega Sainz

1.1.1 (2017-09-20)
------------------
* Change maintainer tags to MAS Robotics
* Generate changelogs for mdr_manipulation
* Contributors: Argentina Ortega Sainz

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------

1.0.0 (2017-04-11)
------------------
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
* [tactile tools] add CATKIN_ENABLE_TESTING
* remove not existing script from install command
* Merge pull request #40 from jsanch2s/grasp_evaluators_update
  change logging level from 'info' to 'debug'
* change logging level from 'info' to 'debug'
* Merge pull request #33 from jsanch2s/tactile_tools
  move 'mdr_tactile_tools' package to 'mdr_manipulation'
* move 'mdr_tactile_tools' package to 'mdr_manipulation'
* Contributors: Frederik Hegger, Jose Sanchez, Alex Moriarty, Sven Schneider
