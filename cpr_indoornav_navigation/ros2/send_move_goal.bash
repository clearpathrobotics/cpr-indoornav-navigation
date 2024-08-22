#!/bin/bash

if [ $# != 3 ];
then
  echo "Usage: send_move_goal.sh X Y YAW"
  exit 1
fi

if [ -z "$ROS2_DISTRO" ];
then
  ROS2_DISTRO="galactic"
fi

export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///opt/ros/noetic/share/cpr_indoornav_base/config/cyclone_dds.xml

source /opt/ros/$ROS2_DISTRO/setup.bash

ros2 run clearpath_api_examples send_move_goal rocksteady $1 $2 $3