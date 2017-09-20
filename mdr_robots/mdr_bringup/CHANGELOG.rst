^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mdr_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.1.0 (2017-09-18 18:15)
------------------------

1.0.1 (2017-09-18 18:04)
------------------------
* rm sicktoolbox dependencies
* rm unused launchers
* rm volksbot launch files
* rm mdr_hardware_config
* Contributors: Alex Mitrevski

1.0.0 (2017-04-11)
------------------
* Fix openni2 launch file
* Do not launch the USB cam nice it crashes quite often and it is not used currently
* Fix ending include tag
* Specifiy location of env.sh as parameter
* Remove commented openni include
  since this is already included in the cob bringup
* Change from hydro to indigo in env script
* Launch Microsoft LifeCam 1080p
* Reduce resolution and frame rate
* Comment launch of script server and openni2
  since it is already launched within the cob bringup
* Remove unused arguments
* Cleanup dependencies
* [volksbot] upload head joint configuration
* Publish tf for 3d cam
* Keep openni launch file arguments consistent
* Use the normal openni version for the Kinect
* Add launch file for microsoft life cam with default parameters
* Do not launch stemmer cam for now since it is not working properly
* Enable depth registration by default
* Fix indentation
* Pass missing argument upwards
* Remove old cam3d file
* Do not publish tf from openni since it is already specified in the urdf
* Add machine argument to the include of the openni
* Update openni launch files
* Do not start kinect and rgb cam
* Fix emergency stop mapper
* Enable depth registration by default
* Merge branch 'hydro' of github.com:mas-group/mas_domestic_robotics into hydro
* Remove kinect_led dependency
* Add keyboard teleoperation lauch file
* Remove joypad launch file
* Add RViz config for the Care-O-bot
* Remove mcr\_ prefix and add namespace
* Add stemmer launch file
* Add stemmer cam to cob launch file
* Remove door status from bringup
* Add env-loader
* Move file one folder up
* Change to hydro setup
* Merge branch 'hydro' of github.com:mas-group/mas_domestic_robotics into hydro
  Conflicts:
  repository.debs
* Rename kinect launch file
* Update to new openni2 version
* Correct prefix of bringup package
* Fixed name changed of volksbot_motorcontroller
* Add tool bringups
* Add cam3d bringup
* Add robot bringups
* Add bringup components
* Add bringup package
* Contributors: Frederik Hegger, Hans Wurst, Shehzad Ahmed, Sven Schneider, Jose Sanchez, Matthias Füller
