cmake_minimum_required(VERSION 3.0.2)
project(referee_system_race)

find_package(catkin REQUIRED)

catkin_package()

# 安装二进制可执行文件（由 pyinstaller_build.sh 生成并复制到 bin/）
install(PROGRAMS
  bin/referee_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# images 已嵌入 referee_node 二进制，不再单独安装
install(DIRECTORY launch materials
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ARRIVAL_TOPIC.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
