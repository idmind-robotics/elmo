#! /bin/bash


source /opt/ros/melodic/setup.bash
source /home/idmind/elmo/catkin_elmo/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=elmo
export DISPLAY=0:0
roslaunch elmo elmo_nosudo.launch --wait

