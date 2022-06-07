==========
Robotic applications 
README file
==========

start by running roscore
in a separate tab run the launch file -> roslaunch olk11_ros360 room.launch
wait for gazebo, rviz and contours to load
in a third tab run the python script -> rosrun olk11_ros360 Colourlocate.py

code runs, placing all blue cubes in bin (when cubes dont fall out of gripper, if so... delete cube from gazebo)

code exists by itself when no cubes are found after a full scan of environment

