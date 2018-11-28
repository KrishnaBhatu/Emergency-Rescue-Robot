[![Build Status](https://travis-ci.org/KrishnaBhatu/Emergency-Rescue-Robot.svg?branch=master)](https://travis-ci.org/KrishnaBhatu/Emergency-Rescue-Robot)
[![Coverage Status](https://coveralls.io/repos/github/KrishnaBhatu/Emergency-Rescue-Robot/badge.svg?branch=master)](https://coveralls.io/github/KrishnaBhatu/Emergency-Rescue-Robot?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<h1 align = "center">
  DAREDEVIL
</h1>
ENPM808X-Final Project (Frontier Exploration Robot: Explore an unknown space)

## Team Members

 - Krishna Bhatu [Github Link](https://github.com/KrishnaBhatu)
 - Siddhesh Rane [Github Link](https://github.com/srane96)

## Project Overview

This project implements the frontier exploration task where the robot has to react in an emergency situation and lead the person to the exit of the building. The project is just a simulation of the robot in Gazrbo which will show the autonomous exploration of the unknown environment and finding the exit of the building by following the exit signs.

While travelling and exploring the unknown environment, the robot will face static obstacles which it has to avoid and also the robot has to interpret the direction of the exit signs and turn accordingly. Robot moves in the direction of the exit sign until an obstacle is detected by the laser sensor. Robot then follows the boundaryu of the obstacle until sign is visible again. Also, for the demonstration purpose, the direction of the exit signs are interpreted by different colors.

## Application

Emergency situations are chaotic, and it is more difficult for the visually impaired people to face such situation which require the evacuation of the building. So, the product  could be installed in every building, which will be triggered into action as a reaction to emergency situation to provide aid. This product will look for emergency exit signs and guide the person to the exit as soon as possible.

## Development using Solo Iterative Process (SIP) and Test-Driven Development (TDD)

In this project, solo-iterative process is used where first the product backlog is created. Then the higest priority requirnments are selected and assigned at the top of the TODO task. In the project backlog, estimate time of completion was alloted to every task. Actual time of completion was compared with the estimated time and based on that, the time allotement of the future tasks is modified.

After the planning is done, the UML folw diagram and the UML class diagram of the software are developed. Based on the UML diagrams the unit test classes are written. Then the stub classes are written with the functions matching the test cases. Thus the coverage of the software is maintained.

Following is the link to the spreadsheet that contains the detailed entries of the product backlog, time log, error log and release backlog - [link](https://docs.google.com/spreadsheets/d/1O63iHQKQJ4rw-KZTfBKcTz6vqTiGLA_Ci8IV8-E5Vwg/edit?ts=5bfdd247#gid=0)

## Sprint Planning and Review

Following is the link to google doc with the sprint planning and review - [link](https://docs.google.com/document/d/1R3kuxY5z7W4jexqUmMMtAAA1UcctKT3b8epEXvo6OiU/edit?ts=5bfe07c4)

## Dependencies

 - Ubuntu 16.04
 - ROS Kinetic
 - Gazebo (if not already installed with ROS)
 - Turtlebot
 - python (if not already installed)
 - Googletest
 - OpenCV
 - rviz (optional)(if you want the visualization of the rostopics)
 - catkin_pkg (if not already installed while installing ROS)

For installing catkin_pkg and directly adding it to PYTHONPATH.
```
pip install catkin_pkg
```
Check if the catkin_pkg path is added to PYTHONPATH by using the following
command
```
echo $PYTHONPATH
```

## Installation

ROS Installation:
Install the ROS Kinetic for Ubuntu and it's dependencies using the [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Gazebo Installation:
For the installation of Gazebo follow the [link](http://gazebosim.org/tutorials?tut=install_ubuntu)

To download the Turtlebot simulink model use the following command;
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

To install Google Test follow the given [link](https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/).

Install OpenCV on Linux by following the instructions on this [link](https://github.com/kyamagu/mexopencv/wiki/Installation-(Linux,-Octave,-OpenCV-3))

## Build Instructions

To make the catkin workspace:

--skip command 1 (sudo rm -R ~/catkin_ws) if no such folder is present in home directory

Open a terminal and type the following command:

```
sudo rm -R ~/catkin_ws
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
cd src/
git clone https://github.com/KrishnaBhatu/Emergency-Rescue-Robot
cd ..
catkin_make
```

Now the package is ready to use

## Run Instructions
TODO

## Run Test Instructions
TODO

## Video Presentation
TODO


 

