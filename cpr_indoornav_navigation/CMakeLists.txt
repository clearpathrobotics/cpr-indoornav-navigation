cmake_minimum_required(VERSION 3.0.2)
project(cpr_indoornav_navigation)
find_package(catkin REQUIRED)

catkin_python_setup(
)

catkin_package(
)

include_directories(
)

catkin_install_python(PROGRAMS
  scripts/move_goal_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch ros2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
