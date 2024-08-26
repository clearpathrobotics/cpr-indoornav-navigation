cmake_minimum_required(VERSION 3.0.2)
project(cpr_indoornav_navigation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    actionlib_msgs
    std_msgs
)

add_message_files(
  FILES
    Marker.msg
    Mission.msg
    Place.msg
    Task.msg
)

add_service_files(
  FILES
    GetMarkers.srv
    GetMissions.srv
    GetPlaces.srv
)

add_action_files(
  DIRECTORY
    action
  FILES
    ExecuteMission.action
    MoveToLocation.action
    MoveToMarker.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
)

include_directories(
)