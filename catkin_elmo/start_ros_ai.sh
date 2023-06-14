#! /bin/bash


source /opt/ros/melodic/setup.bash
source /home/idmind/elmo/catkin_elmo/devel/setup.bash
source /home/idmind/edenai.sh
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=elmo
export DISPLAY=0:0
roslaunch idmind_edenai main.launch

