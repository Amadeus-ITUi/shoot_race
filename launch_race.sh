#!/bin/bash
# 启动仿真、裁判系统、环形靶识别、AR 码识别（使用 release 内容）
# 用法: ./launch_race.sh  或  bash launch_race.sh
# 使用 gnome-terminal 多标签方式启动，每个节点独立窗口

# 定位 release 目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/release/setup.bash" ]; then
  RELEASE_DIR="$SCRIPT_DIR/release"
else
  RELEASE_DIR="$SCRIPT_DIR"
fi
SETUP_CMD="source /opt/ros/noetic/setup.bash; source /usr/share/gazebo/setup.sh; source $RELEASE_DIR/setup.bash"

gnome-terminal --window -e 'bash -c "exec bash"' \
--tab -e "bash -c \"sleep 1; $SETUP_CMD; roslaunch abot_model gazebo_world_2026.launch; exec bash\"" \
--tab -e "bash -c \"sleep 10; $SETUP_CMD; roslaunch target_detector target_detector.launch; exec bash\"" \
--tab -e "bash -c \"sleep 12; $SETUP_CMD; roslaunch target_detector alvar_detection.launch; exec bash\""
