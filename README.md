[![pipeline status](https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/badges/indigo/pipeline.svg)](https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/commits/indigo)


## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Indigo: Ubuntu 14.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make.

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.


- ROS Indigo - http://wiki.ros.org/indigo/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc!


### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://wiki.ros.org/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters").


## Set up a catkin workspace

    source /opt/ros/indigo/setup.bash
    mkdir -p ~/indigo/src; cd ~/indigo/src
    catkin_init_workspace
    catkin build

## Clone and compile the MAS domestic robotics software
First of all you have to clone the repository.

    cd ~/indigo/src;
    git clone gitgate@mas.b-it-center.de:mas-group/mas_domestic_robotics.git

Then go on with installing further external dependencies:

    cd ~/indigo/src/mas_domestic_robotics
    ./repository.debs

    source ~/indigo/devel/setup.bash

The last command should be added to the ~/.bashrc file so that they do not need to be executed everytime you open a new terminal.


And finally compile the repository:

    cd ~/indigo
    catkin build


If no errors appear everything is ready to use. Great job!


### Setting the Environment Variables
#### ROBOT variable
With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line will add the variable to your .bashrc:

     echo "export ROBOT=cob3-1" >> ~/.bashrc
     source ~/.bashrc



#### ROBOT_ENV Variable
The ROBOT_ENV variable can be used to switch between different environments. The following line will add the variable to your .bashrc:

     echo "export ROBOT_ENV=brsu-c069" >> ~/.bashrc
     source ~/.bashrc



## Bring up the robot and it's basic components
### In Simulation

The first time you start the simulation, Gazebo tries to download objects from the internet. Thus, make sure that you have established an internet connection before running the following command.

     roslaunch mdr_bringup_sim robot.launch


In a new terminal you can open the Gazebo GUI to see the environment and the robot

     gzclient


### At the Real Robot

     roslaunch mdr_bringup robot.launch


## Test the arm, hand, torso, tray and head

     roslaunch mdr_bringup dashboard.launch


## Test the base

     roslaunch cob_teleop teleop_keyboard.launch


## Visualize the robot state and sensor data

     rosrun rviz rviz


## Build a map for base navigation

     roslaunch mdr_2dslam 2dslam.launch


## Use autonomous navigation
### Omni-directional navigation

     roslaunch mdr_2dnav 2dnav.launch nav_mode:=dwa

or

     roslaunch mdr_2dnav 2dnav.launch nav_mode:=eband


Click on the menu bar "File -> Open Config", navigate to "~/indigo/src/mas_domestic_robotics" and select the "cob.rviz" file.
