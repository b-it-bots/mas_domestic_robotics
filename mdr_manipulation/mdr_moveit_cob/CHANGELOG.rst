^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_moveit_cob
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [mdr_moveit_cob] Fixed include in move_group.launch
* Contributors: Alex Mitrevski

1.0.0 (2017-04-11)
------------------
* Merge branch 'moveit-client-launch' into 'indigo'
  [moveit] add moveit client launch and config
  See merge request !18
* Merge branch 'perceive_table' into 'indigo'
  Merge Perceive Table Skill to indigo
  See merge request !17
* Include moveit_client, ignore python .swp files
* [moveit cob] regenerate config after changing the arm reference pos
* Merge branch 'catkin_lint' into 'indigo'
  [CATKIN_LINT] fix issues reported by catkin lint
  - Adds missing CATKIN_DEPENDS to catkin_package()
  - Adds missing install()
  - mdr_lab_demo is probably broken: It depends on a package which doesn't exist, replaced with the closest thing (or maybe old package was just renamed)
  See merge request !6
* [CATKIN_LINT] fix issues reported by catkin_lint
* [moveit] disable usage of kinect data
* [moveit] set default planner
* [moveit] rename cob files to use dash instead of underscore
* [moveit] adapt to new controller namespaces
* Update moveit config
* Fix moveit package.xml
* [dependencies] cleanup
* Disable head kinematics plugin (not working at the moment)
* Add lookat joints as passive joint
* Merge pull request #46 from jsanch2s/precision_grasp_poses
  Precision grasp poses
* Integrate feedback
* Add arm poses for 'precision' grasp
* Update cob3-1 urdf in moveit config
* Increase jiggle to get the torso out of collision with the arm after initialization
* Set correct velocity and acceleration values for the torso
* Reduce the arm's velocity and acceleration
* Use the controller's configuration for the tray's planner
* Generate planner configuration for tray and torso
* Add new pre-defined poses for the torso and the head
* Add the controller configuration for the real robot
* Separate the torso and the head description
* Adapted the moveit controller configuration to the real robot
* Add controller configuration to let moveit move the components
* Add urdf model of cob for the setup assistant
* Add tray to moveit
* Added pre-defined poses from ipa's cob repositories
* Re-generated the moveit configuration for the care-o-bot and renamed the package
* Contributors: Frederik Hegger, Minh Nguyen, Santosh Thoduka, Sven Schneider, Jose Sanchez, Alex Moriarty
