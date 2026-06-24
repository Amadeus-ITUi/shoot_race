cmake_minimum_required(VERSION 3.0.2)
project(abot_planar_move_plugin)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  std_srvs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_planar_move_fixed
  CATKIN_DEPENDS gazebo_ros geometry_msgs nav_msgs roscpp tf
)

add_library(gazebo_ros_planar_move_fixed src/gazebo_ros_planar_move_fixed.cpp)
add_dependencies(gazebo_ros_planar_move_fixed ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_planar_move_fixed ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS gazebo_ros_planar_move_fixed
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
