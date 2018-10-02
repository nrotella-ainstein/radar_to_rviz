#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/Desktop/catkin_ws/devel/setup.bash

roslaunch radar_to_rviz run_car_demo.launch
#roslaunch /home/nvidia/Desktop/catkin_ws/src/radar_to_rviz/launch/run_car_demo.launch
