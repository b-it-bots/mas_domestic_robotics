# Setup

## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Kinetic: Ubuntu 16.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control

### Install Git
Install the Git core components and some additional GUI's for version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this, you need to open a new terminal. First you need to tell git your name so that it can properly label the commits you make:

     git config user.name "Your Name Here"

Git also saves your email address into the commits you make:

     git config user.email "your-email@youremail.com"

### GIT Tutorial

If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/

## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.

- ROS Kinetic - http://wiki.ros.org/kinetic/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc!

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://wiki.ros.org/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters").


## Clone and compile the MAS domestic robotics software
b-it-bots members can use [these instructions](https://github.com/b-it-bots/dev-env#setup) to setup a complete development environment for all our robots.

For external users, the following instructions should get you a working system:

1. Set up a catkin workspace

  ```
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
  wstool init src
  wstool merge -t src https://raw.githubusercontent.com/b-it-bots/mas_domestic_robotics/devel/mas-domestic.rosinstall
  ```
2. Get the code and dependencies

  ```
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
  ```
3. Build the code

  ```
  catkin build
  ```

If you encounter any problems, please check the list of [issues](https://github.com/b-it-bots/mas_domestic_robotics/issues) and open a new one if you don't see a discussion of the problem there.
