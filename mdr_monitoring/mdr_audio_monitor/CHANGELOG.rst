^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_audio_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Merge branch 'groovy_catkin' of github.com:b-it-bots/mas_domestic_robotics into groovy_catkin
* Added message runtime dependency
* Replace generate_messages by run_depends + CATKIN_DEPENDS
* [mdr_audio_monitor] change node name (remove brsu), clean up empty space
* Merge branch 'groovy_catkin' of github.com:b-it-bots/mas_domestic_robotics into groovy_catkin
* Make package.xml xml compliant
* Removed rosbuild's mainpage.dox
* Fix names
* Cleaned up cmake script
* Updated package.xml of the audio monitor
* Removed import roslib, added rospy
* Added mdr_audio_monitor
* Contributors: Frederik Hegger, Rhama Dwiputra, Sven Schneider, Matthias Füller, Alex Moriarty
