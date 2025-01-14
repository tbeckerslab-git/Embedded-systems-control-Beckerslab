#!/bin/bash
export SOFA_ROOT=/home/glorbs/sofa/build
export PYTHONPATH=/home/glorbs/sofa/build/lib/python3/site-packages

source /opt/ros/noetic/setup.bash
source /home/glorbs/soft_robot_cv/devel/setup.bash

cd /home/glorbs/soft_robot_cv
catkin_make

# Specify the package name and launch file name
# TODO: pass in command line arguments for cam or not
simulated=$1
detection_method=$2 

if [ "$simulated" == "sim" ] && [ "$detection_method" == "aruco" ]; then
   # Run the roslaunch command
    roslaunch soft_robot_cv simple_cam.launch &

    #Start the sofa sim
    python3 /home/glorbs/sofa/external_directories/SofaSoftGripper/src/gripper/gripper_scene.py


else
   echo "not sim"
fi




