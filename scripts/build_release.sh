#!/bin/bash
# 射击比赛仿真系统 - 比赛版打包脚本
# 将 Python 节点打包为二进制，生成 release/ 可部署文件夹（无源码）
# 用法: cd /home/guo/shoot_race && ./scripts/build_release.sh
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RELEASE="$WS/release"
cd "$WS"

echo "=== 1. 检查 ROS 环境 ==="
[ -z "$ROS_DISTRO" ] && { echo "请先 source /opt/ros/noetic/setup.bash"; exit 1; }

echo "=== 2. 生成 shoot_sim 嵌入参数并打包 ==="
python3 scripts/generate_shoot_sim_release_params.py
./src/shoot_sim/tools/pyinstaller_build.sh

echo "=== 3. 从源码同步 release 配置并打包 referee 节点 ==="
python3 scripts/sync_release_packages.py
python3 scripts/generate_release_params.py
./src/referee_system/tools/pyinstaller_build.sh

echo "=== 4. 临时加入保密包到 src（用于 catkin 编译）==="
cp -r "$WS/release_packages/referee_system_race" "$WS/src/"
cp -r "$WS/release_packages/shoot_sim_race" "$WS/src/"

echo "=== 5. 编译主工作空间 ==="
catkin_make
catkin_make install

echo "=== 6. 移除临时 race 包 ==="
rm -rf "$WS/src/referee_system_race" "$WS/src/shoot_sim_race"

echo "=== 7. 生成 release/ 部署文件夹 ==="
./scripts/create_release_workspace.sh "$RELEASE"

echo ""
echo "=== 完成 ==="
echo "部署文件夹: $RELEASE"
echo "该文件夹为纯 install 目录，无任何源码（含二进制节点、编译后 .so、配置）。"
echo "任务板图片已嵌入 referee 二进制，不可替换。"
echo ""
echo "在目标机运行:"
echo "  cd release  # 或复制后的目录"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source setup.bash"
echo "  roslaunch abot_model gazebo_world_2026_race.launch"
