cmake_minimum_required(VERSION 3.0.2)
project(shoot_sim_race)

find_package(catkin REQUIRED)

catkin_package()

# 安装二进制可执行文件（由 pyinstaller_build.sh 生成并复制到 bin/）
install(PROGRAMS
  bin/shoot_sim_node
  bin/shoot_targets_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY materials
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
