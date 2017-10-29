^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_simple_grasp_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2017-10-29)
------------------

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
* Cleanup dependencies
* [simple grasp planner] remove comments and set fields correctly
* Merge branch 'fix_simple_grasp_planner_test_2' into 'hydro'
  Fix simple grasp planner test 2
  Fix of simple_grasp_planner test to address issue #8: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/issues/8
  See merge request !2
* Include tests in CMakeLists.txt
* Check all tests in launch folder for catkin_make run_tests
* Remove unused import statements
* Add file to implement integration test
* Change path of test files to fix ImportError
* Added simple grasp planner
* Contributors: Frederik Hegger, Sven Schneider, Jose Sanchez, Alex Moriarty
