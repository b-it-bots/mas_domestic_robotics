^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_pick_and_place
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------
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
  I've separated the commits so you can cherry pick the ones you want.
  You may not like f7e2b9412, where I changed repository.rosinstall to get the version from ipa320 instead of mas-group, I just did that to save time.
  @fhegge2m & @sschne2m
  See merge request !4
* Fix Python Packages for latest Catkin
  see: https://github.com/ros/catkin/issues/756
* [pick and place] add missing catkin_python_setup
* [dependencies] cleanup
* Merge branch 'hydro' of mas.b-it-center.de:mas-group/mas_domestic_robotics into hydro
* [simple pick and place] remove comments
* use the mas base navigation launch file in the pick and place scenario
* Merge branch 'hydro' of github.com:mas-group/mas_domestic_robotics into hydro
  Conflicts:
  mdr_robots/mdr_bringup/components/openni2.launch
* fixed pick and place scenario
* fixed pick and place launch file
* fixed syntax error
* add launch file for pick and place scenario
* update pick and place scenario
* added pick and place scenario
* Contributors: Frederik Hegger, Hans Wurst, Sven Schneider, demo@c069-workstation-1, demo@cob3-1-pc1, moriarty
