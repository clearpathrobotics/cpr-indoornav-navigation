#!/bin/bash
# See https://docs.ottomotors.com/Archive/2-26/out/en/30495-31257-advanced-attachment-acquiring-real-time-robot-and-job-information.htm

if [ -z "$ROS2_DISTRO" ];
then
  ROS2_DISTRO="galactic"
fi

export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(catkin_find cpr_indoornav_base config/cyclone_dds.xml --first-only)

source /opt/ros/$ROS2_DISTRO/setup.bash