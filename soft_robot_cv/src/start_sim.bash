#!/bin/bash

# Specify the package name and launch file name
package_name="your_package_name"
launch_file_name="your_launch_file.launch"

# Run the roslaunch command
roslaunch soft_robot_cv simple_cam.launch &

#Start the sofa sim
/home/glorbs/sofa/build/bin/runSofa /home/glorbs/sofa/external_directories/SofaSoftGripper/src/gripper/gripper_scene.py

