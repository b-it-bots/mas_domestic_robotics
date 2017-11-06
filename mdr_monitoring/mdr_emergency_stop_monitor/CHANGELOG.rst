^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_emergency_stop_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2017-11-04)
------------------

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
* Update dependencies to mdr_speech_msgs
* Contributors: Argentina Ortega Sainz

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
* EmergencyStopState has moved to cob_msgs
  - In 0.5.6 the EmergencyStopeState was moved:
  http://docs.ros.org/indigo/changelogs/cob_relayboard/changelog.html
* Update dependencies of emergency stop monitor
* Use std_msg/String msg instead of custom msg
* Merge branch 'hydro' of github.com:mas-group/mas_domestic_robotics into hydro
* Removed old documentation file
* Merge remote-tracking branch 'origin/hydro' into hydro_merge
* Add catkin include directories to emergency stop monitor
* Cleaned up cmake script
* Merge branch 'groovy_catkin' of github.com:b-it-bots/mas_domestic_robotics into groovy_catkin
* Fix indentation in emergency stop monitor
* Merge branch 'groovy_catkin' of github.com:b-it-bots/mas_domestic_robotics into groovy_catkin
  Conflicts:
  mdr_monitoring/mdr_emergency_stop_monitor/package.xml
  mdr_monitoring/mdr_emergency_stop_monitor/ros/src/mdr_emergency_stop_monitor.cpp
* Reduce queue size
* Adjust header include
* Remove url
* ROS info should only be used with literals or format strings
* Fixed dependencies
* Fixed message generation dependencies
* Cleaned up package.xml
* Make package.xml xml compliant
* Use more consistent coding style
* Renamed emergency stop node
* Install executable only in BIN folder
* Switch from Say service to Say topic
* Moved from mcr_speech_srvs to _msgs
* Added mdr_emergency_stop_monitor
* Contributors: Frederik Hegger, Sven Schneider, Matthias Füller, Alex Moriarty
