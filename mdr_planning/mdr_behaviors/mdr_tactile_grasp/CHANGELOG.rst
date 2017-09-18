^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_tactile_grasp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* Merge pull request #21 from jsanch2s/sdh_monitors
  Add SDH monitors
* Add package dependencies for the monitors node and create a setup file
* Add monitors node to the package
* Merge pull request #19 from jsanch2s/fix_commit3
  Add structure of tactile grasp
* Contributors: Frederik Hegger, Jose Sanchez, Alex Moriarty, Sven Schneider
