^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_mockup_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2017-10-29)
------------------

1.1.1 (2017-09-20)
------------------
* Change maintainer tags to MAS Robotics
* Generate changelogs for mdr_simulation
* Contributors: Argentina Ortega Sainz

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------
* Update dependencies to mdr_speech_msgs
* Contributors: Argentina Ortega Sainz

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
* Update dependencies of mockup gui
* Use std_msg/String msg instead of custom msg
* Changed service to topics (SetRecognizedObjects, Doorstate)
* Add just script names
* Replace generate_messages by run_depends + CATKIN_DEPENDS
* Clean up comments
* Initial version of mcr_mockup_gui
* Contributors: Frederik Hegger, Sven Schneider, Matthias Füller, Alex Moriarty
