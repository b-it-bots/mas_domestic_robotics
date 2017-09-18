^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_lwr_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2017-04-11 10:44:12 +0200)
---------------------------------
* Fix CMakeLists.txt
  * Correct install
  * Remove link directories
  * Add missing find_package
  * Use only one include_directories
* Cleanup dependencies
* Merge branch 'hydro' of github.com:mas-group/mas_domestic_robotics into hydro
  Conflicts:
  mdr_robots/mdr_bringup/components/openni2.launch
* Fixed inverse kinematics so that it uses the closest solution to the provided seed state
* Re-generated the lwr kinematics plugin and moved it to the kinematics meta-package
* Contributors: Frederik Hegger, Sven Schneider
