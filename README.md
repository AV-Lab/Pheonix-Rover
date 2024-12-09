# Pheonix Rover

This repository contains all the code used to run the Pheonix Rover.\
***Note:*** These files are already present in the Jetson Xavier, this serves as a reference and a backup.

## Index

- Arduino Code
- ROS2 Foxy Packages
- Quick Launch
- Operation Guide
- Launching individual modules
- Known Issues

## Arduino code

Within the Arduino_Code folder you will find the arduino code that is currently uploaded on the arduino inside the bot.

## ROS2 Foxy Packages

The src folder has the ROS2 foxy packages, each package serves a specific function:

**1) pheonix_rover**


   
**2) pheonix_rover_description**

Contains the URDF, transforms, ekf config file, SLAM toolbox config file(move to pheonix_rover), and the rviz configuration file (with a launch file which opens all these components).

**3) urg_node2**

ROS2 driver for the Hokuyo UTM-30LX Lidar.
I have altered the parameters to fit this context the original git for the package can be found here.

**4) bluespace_ai_xsens_ros_mti_driver**

ROS2 driver for the xsens MTi-30 IMU.
Similar to the urg_node2 package, I have changed a few things in the configuration to make it compatible, the original git is here.

## Quick Launch

## Operation Guide

1) Manually

2) Using Nav2

## Launching individual submodules

## Known Issues

