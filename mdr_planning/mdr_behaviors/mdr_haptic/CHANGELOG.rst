^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_haptic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Set default behavior to DRIVE
* Correct script name in install commmand
* Remove src file from install command
* Correct indentation
* Update dependencies of haptic package
* Remove debug publisher
* Adjust base cmd vel and wrench topic
* Merge branch 'hydro' of github.com:b-it-bots/mas_domestic_robotics into hydro
* Adjust params to new namespace
* Remove mdr\_ prefix from node name
* Correct naming and add namespace to launch file
* [mdr_haptic] added haptic guidance
* Contributors: Frederik Hegger, Rhama Dwiputra, Jose Sanchez, Matthias Füller, Alex Moriarty
