# uwb_localization
Ultra-wideband localization from autonomous driving summer 2018 internship
Author: Jonathan Hoff

This repository contrains the C++ implementation in ROS for localizing an autonomous vehicle using IMU, GNSS, and ultra-wideband (UWB) sensors and a C++ driver for the IndoTraq UWB system.

This module is integrated with the Apollo autonomous driving framework. It also communicates with the robot_localization ROS package in order to run an EKF to fuse the IMU data with either the GNSS or the UWB data. The module contains switching logic for selecting either GNSS or UWB as data to use. UWB is used if the car enters the UWB box, and it switches back to GNSS when it exits. 
