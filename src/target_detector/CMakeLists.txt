cmake_minimum_required(VERSION 3.0.2)
project(target_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
  ar_track_alvar_msgs
)

add_message_files(FILES MarkerPixel.msg MarkerPixelArray.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime ar_track_alvar_msgs)

catkin_install_python(PROGRAMS
  scripts/target_detector_node.py
  scripts/ar_pose_to_pixel_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES msg/MarkerPixel.msg msg/MarkerPixelArray.msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
