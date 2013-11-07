## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Hydro: Ubuntu 12.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make. We use the email address to associate your commits with your GitHub account:

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.


- ROS Hydro - http://www.ros.org/wiki/Hydro/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc!
  

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://www.ros.org/wiki/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 


## Set up a catkin workspace

    mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    
## Clone and compile the MAS domestic robotics software
First of all you have to clone the repository.

    cd ~/catkin_ws/src;
    git clone git@github.com:mas-group/mas_domestic_robotics.git

Then go on with installing further external dependencies:
       
    cd ~/catkin_ws/src/mas_domestic_robotics
    ./repository.debs
    
    source ~/catkin_ws/devel/setup.bash

And finally compile the repository:

    cd ~/catkin_ws
    catkin_make


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

     roslaunch mdr_bringup_sim robot.launch
     
     
In a new terminal you can open the Gazebo GUI to see the environment and the robot

     rosrun gazebo_ros gzclient


### At the Real Robot

     roslaunch mdr_bringup robot.launch


## Test the arm, hand, torso, tray and head

     roslaunch mdr_bringup dashboard.launch


## Test the base

     roslaunch mdr_brinup teleop_keyboard.launch


## Visualize the robot state and sensor data

     rosrun rviz rviz
     

Click on the menu bar "File -> Open Config", navigate to "~/catkin_ws/src/mas_domestic_robotics" and select the "cob.rviz" file.

     
