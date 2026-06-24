#!/bin/bash
# 创建 release/ 部署文件夹：纯 install 目录，无源码
# 含：编译后的 .so、二进制节点（图片已嵌入）、配置
# 用法: ./scripts/create_release_workspace.sh [目标目录]
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-$WS/release}"

echo "源工作空间: $WS"
echo "目标目录: $DEST"

# 清空并复制 install 目录（无任何源码）
# 注意：cp /* 不复制隐藏文件，.catkin 必须单独复制（ROS 靠它识别工作空间并设置 ROS_PACKAGE_PATH）
rm -rf "$DEST"
mkdir -p "$DEST"
cp -a "$WS/install"/* "$DEST/"
[ -f "$WS/install/.catkin" ] && cp -a "$WS/install/.catkin" "$DEST/"

# 移除 referee 包内 images（已嵌入二进制，防止替换）
RM_IMAGES="$DEST/share/referee_system_race/images"
[ -d "$RM_IMAGES" ] && rm -rf "$RM_IMAGES"

# 移除 materials/textures 中的任务板图片（运行时由 referee 从嵌入二进制写入，防止替换）
RM_TEX="$DEST/share/referee_system_race/materials/textures"
[ -d "$RM_TEX" ] && rm -f "$RM_TEX"/*.jpg "$RM_TEX"/*.jpeg "$RM_TEX"/*.png 2>/dev/null || true

# 替换 referee launch 为最小版：参数已嵌入二进制，参赛者修改 launch 无效
REF_LAUNCH="$DEST/share/referee_system_race/launch/referee.launch"
if [ -f "$REF_LAUNCH" ]; then
  cat > "$REF_LAUNCH" << 'REFLAUNCH'
<launch>
  <!-- 裁判系统：参数已嵌入二进制，launch 仅启动节点，修改无效 -->
  <node pkg="referee_system_race" type="referee_node" name="referee" output="screen" />
</launch>
REFLAUNCH
fi

# 替换 shoot_sim_race launch 为最小版：参数已嵌入二进制，参赛者修改无效
for f in shoot_sim.launch shoot_targets.launch; do
  L="$DEST/share/shoot_sim_race/launch/$f"
  if [ -f "$L" ]; then
    if [ "$f" = "shoot_sim.launch" ]; then
      cat > "$L" << 'SHOOTSIM'
<launch>
  <!-- race 版：参数已嵌入二进制，launch 仅启动节点，修改无效 -->
  <node pkg="shoot_sim_race" type="shoot_sim_node" name="shoot_sim" output="screen" />
</launch>
SHOOTSIM
    else
      cat > "$L" << 'SHOOTTARGETS'
<launch>
  <!-- race 版：参数已嵌入二进制，launch 仅启动节点，修改无效 -->
  <node pkg="shoot_sim_race" type="shoot_targets_node" name="shoot_targets" output="screen" />
</launch>
SHOOTTARGETS
    fi
  fi
done

# 移除旧版 launch（若存在，兼容历史构建）
[ -f "$DEST/share/abot_model/launch/gazebo_world_2026_confidentiality.launch" ] && rm -f "$DEST/share/abot_model/launch/gazebo_world_2026_confidentiality.launch"

# 移除非 race 包（部署仅用 *_race 二进制版，不含源码）
for pkg in referee_system shoot_sim; do
  [ -d "$DEST/share/$pkg" ] && rm -rf "$DEST/share/$pkg"
  [ -d "$DEST/lib/$pkg" ] && rm -rf "$DEST/lib/$pkg"
  [ -f "$DEST/lib/pkgconfig/$pkg.pc" ] && rm -f "$DEST/lib/pkgconfig/$pkg.pc"
done

# 移除 C++ 插件头文件（仅保留 .so，无源码）
[ -d "$DEST/include/abot_planar_move_plugin" ] && rm -rf "$DEST/include/abot_planar_move_plugin"

# 修复 world 文件中的绝对路径为相对路径（确保部署到其他电脑可加载）
RACE_WORLD="$DEST/share/abot_model/worlds/race.world"
if [ -f "$RACE_WORLD" ]; then
  sed -i 's|/home/[^/]*/shoot_race[^"]*worlds/\.\./meshes/|../meshes/|g' "$RACE_WORLD"
  sed -i 's|/home/[^/]*/shoot_race[^"]*worlds/\.\./shoot_race_world/|../shoot_race_world/|g' "$RACE_WORLD"
fi

# 打包 OpenCV 3.4 库（ar_track_alvar 依赖，目标机可能无 /usr/local/lib 的 OpenCV 3.4）
OCV34_DIR="/usr/local/lib"
OCV34_LIBS="libopencv_calib3d libopencv_flann libopencv_imgproc libopencv_core libopencv_highgui libopencv_features2d libopencv_imgcodecs"
if [ -f "$OCV34_DIR/libopencv_calib3d.so.3.4" ]; then
  mkdir -p "$DEST/lib/opencv34"
  for lib in $OCV34_LIBS; do
    for f in "$OCV34_DIR"/${lib}.so*; do
      [ -e "$f" ] && cp -a "$f" "$DEST/lib/opencv34/"
    done
  done
  echo "[INFO] 已打包 OpenCV 3.4 库到 lib/opencv34（ar_track_alvar 依赖）"
  # 在 setup.bash 末尾追加 LD_LIBRARY_PATH，使 ar_track_alvar 可加载
  echo '[ -d "$_CATKIN_SETUP_DIR/lib/opencv34" ] && export LD_LIBRARY_PATH="$_CATKIN_SETUP_DIR/lib/opencv34:$LD_LIBRARY_PATH"' >> "$DEST/setup.bash"
else
  echo "[WARN] 未找到 /usr/local/lib 的 OpenCV 3.4，ar_track_alvar 在目标机可能报 libopencv_calib3d.so.3.4 缺失"
fi

# 便携启动脚本：根据脚本所在目录自动 source，复制到任意电脑可用
cat > "$DEST/run.sh" << 'RUNSH'
#!/bin/bash
RELEASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$RELEASE_DIR/setup.bash"
exec roslaunch abot_model gazebo_world_2026_race.launch "$@"
RUNSH
chmod +x "$DEST/run.sh"

# 部署说明
cat > "$DEST/README.md" << 'READMEEOF'
# 射击比赛仿真 - 部署包

本文件夹为比赛部署包，**无任何源码**：
- 节点为二进制可执行文件
- 任务板图片已嵌入 referee 二进制，不可替换
- 可直接复制到其他 Ubuntu 20.04 + ROS Noetic 机器运行。

## 运行

**方式一（推荐）**：直接运行启动脚本，复制到任意电脑可用：

```bash
cd /path/to/release
./run.sh
```

**方式二**：手动 source 后启动：

```bash
cd /path/to/release
source /opt/ros/noetic/setup.bash
source ./setup.bash
roslaunch abot_model gazebo_world_2026_race.launch
```

若报错 `is neither a launch file in package [abot_model]`，说明未正确 source setup.bash。

## 裁判系统 - 参赛者话题

参赛方需发布以下话题与裁判交互：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/shoot_race/spawn_task_board` | `std_msgs/String` | 比赛开始时发布，格式 `target2,target3,wheel_target`（如 `c,g,3`），触发任务板生成、靶子生成；3 个靶子模型加载完成后开始计时 |
| `/shoot_race/arrival` | `std_msgs/String` | 到达任务点或终点时发布，取值 `shoot_1`、`shoot_2`、`shoot_3`、`finish` |

障碍区域 A/B 由裁判按机器人位姿自动判定，无需发布。详细说明见 `share/referee_system_race/ARRIVAL_TOPIC.md`。

## target_detector 靶标检测

需先 source setup.bash，再启动相机或仿真。OpenCV 3.4 已内置于 lib/opencv34/。

**1. AR 码识别**（平移靶、识别任务板等）：`alvar_detection.launch`

```bash
source /opt/ros/noetic/setup.bash
source ./setup.bash
roslaunch target_detector alvar_detection.launch
```

- 订阅 `/camera/image`、`/camera/camera_info`
- 发布 `/ar_marker_pixels`（AR 码像素坐标）

**2. 圆形靶识别**（1 号固定靶靶心）：`target_detector.launch`

```bash
source /opt/ros/noetic/setup.bash
source ./setup.bash
roslaunch target_detector target_detector.launch
```

- 订阅 `/camera/image`、`/camera/camera_info`
- 发布 `/target_center_pixel`（靶心像素坐标）、`/target_detector/debug_image`（可视化）
READMEEOF

echo ""
echo "=== 完成 ==="
echo "部署文件夹: $DEST（纯 install，无源码）"
echo "在目标机运行:"
echo "  cd $DEST"
echo "  ./run.sh"
