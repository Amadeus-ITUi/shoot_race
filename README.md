# Shoot Race — 射击比赛仿真项目

基于 ROS Melodic + Gazebo 的机器人射击竞赛仿真平台，用于"陆地组仿真组 A 题"比赛。

## 项目概述

本项目的比赛场景为 3.6m×3.6m 的场地，机器人从起点基地出发，依次完成三个射击任务点的标靶射击、两个障碍任务点（避障/越障），最后到达终点基地停靠。裁判系统自动计分并生成成绩单。

## 竞赛规则摘要

详见 [陆地组比赛规则.md](陆地组比赛规则.md)。

| 环节 | 最高分 | 说明 |
|------|--------|------|
| 到达射击任务点 1/2/3 | 各 10 分 | 50cm×35cm 区域，按重叠比例计分 |
| 击中 1 号固定靶（环形） | 10 分 | 环数 6~10 |
| 击中 2 号旋转靶 | 10 分 | 迷彩位×1 / 灰色位×2，打中指定叶片得 5×系数 |
| 击中 3 号平移靶 | 10 分 | 迷彩位×1 / 灰色位×2，打中指定区域得 5×系数 |
| 障碍 A（避障） | 10 分 | 两半区各 5 分，触碰障碍 0 分 |
| 障碍 B（越障） | 10 分 | 完全通过区域 |
| 到达终点 | 10 分 | 外圈 5 分 + 内圈 5 分（停靠位），按重叠比例 |
| 技术文档/答辩 | 10 分 | 重复率 > 30% 不参与一二等奖，> 50% 取消资格 |

## 系统架构

```
shoot_race/
├── src/                          # ROS 源码包
│   ├── robot_slam/               # 导航与 SLAM
│   ├── target_detector/          # 视觉检测（靶心 + AR 码）
│   ├── ar_track_alvar/           # [第三方] AR 标签追踪
│   └── abot_planar_move_plugin/  # [第三方] Gazebo 运动控制插件
├── release/                      # 编译产物（含裁判系统 + 射击仿真）
│   └── share/abot_model/         # 机器人 URDF 模型 + 比赛场地 world
├── release_packages/             # 发布包（二进制，不含源码）
│   ├── referee_system_race/      # 裁判系统（PyInstaller 打包）
│   └── shoot_sim_race/           # 射击仿真（靶子生成 + 命中检测）
└── build/ install/               # catkin 构建目录
```

## 各包功能详述

### 1. robot_slam — 导航与建图

- **定位**：AMCL 自适应蒙特卡洛定位，支持 Cartographer
- **建图**：Gmapping 激光 SLAM
- **路径规划**：global_planner（全局）+ TEB/DWA（局部）
- **导航节点**：
  - `navigation_goals.py` — 单目标点导航示例
  - `navigation_multi_goals.py` — 多目标点顺序导航（核心比赛脚本）
  - `navigation_multi_goals111.py` — AR 码识别后分叉路径导航
  - `navigate.cpp` — C++ 版 A→B 顺序导航节点，含语音播报
- **已有地图**：`maps/map.yaml` + `maps/map.pgm`

### 2. target_detector — 目标检测

- `target_detector_node.py` — 霍夫圆变换检测环形靶心像素坐标，支持同心圆验证、时间平滑
- `ar_pose_to_pixel_node.py` — AR 标签 3D 位姿 → 2D 像素坐标投影
- 自定义消息：`MarkerPixel.msg`、`MarkerPixelArray.msg`

### 3. referee_system_race — 裁判系统（仅二进制发布）

- 订阅 `/shoot_targets/hit` 命中消息，按规则计分
- 订阅 `/shoot_race/arrival` 到达消息，按机器人 Gazebo 位姿验证重叠比例
- 定时检测障碍区 A/B 通过状态及碰撞
- 收到 `/shoot_race/spawn_task_board` 后随机选图生成识别任务板（SDF 模型）
- 3 号靶目标区域由任务板图片类别决定（弹药库/装备库/医疗营 → 区域 1/2/3）
- 比赛结束时生成 `match_score_report.txt` 成绩单
- 提供 `~reset` 服务重置计分
- release 版通过 PyInstaller 打包参数，防止参赛者篡改

### 4. shoot_sim_race — 射击仿真（仅二进制发布）

- 生成固定靶、旋转靶、平移靶模型
- 模拟命中检测与射击交互

### 5. 第三方依赖

- **ar_track_alvar**：AR 标签识别与位姿估计
- **abot_planar_move_plugin**：Gazebo 平面移动控制器
- **abot_model**：麦克纳姆轮/阿克曼底盘 URDF + 比赛场地 world

---

## 未完成 / 待完善功能

以下功能根据比赛规则和项目现状分析得出。

### 🔴 关键缺失

| 项目 | 说明 |
|------|------|
| **大模型视觉识别（3 号任务）** | 规则要求通过大模型（VLM）识别军火道具图片（弹药库/装备库/医疗营）并匹配阵地标签码。裁判系统已随机选图并确定正确目标区域，但**参赛方的 AI 识别代码尚未实现**——机器人需自行调用大模型 API 对任务板拍照识别 |
| **射击执行机构** | 命中检测由仿真端的 `shoot_sim_race` 提供，但**机器人端的射击触发逻辑（何时射击、如何瞄准）未实现** |
| **完整比赛流程编排** | 缺少一个整合所有环节的状态机/launch 文件：起点出发 → 射击点 1 → 射击点 2 → 射击点 3 → 障碍 A → 障碍 B → 终点停靠。现有导航脚本是独立运行的示例 |

### 🟡 部分完成

| 项目 | 当前状态 |
|------|----------|
| **起点基地停靠位出发** | 导航脚本参数设置初始位姿，但未根据 90cm×90cm 基地和 50cm×35cm 停靠位做精确对齐 |
| **障碍 A 避障** | 裁判系统已检测碰撞并计分，但**机器人端的避障策略（传感器 + 路径规划）需参赛方自行实现** |
| **障碍 B 越障** | 裁判系统已检测完全通过，但**越障动作本身需要机器人具备相应机械能力或在仿真中模拟** |
| **旋转靶识别与瞄准** | AR 码检测已实现，但 2 号靶为旋转靶，需识别旋转角度并打中指定叶片 |
| **平移靶追踪** | 3 号靶为移动靶，需动态追踪并射击移动目标 |

### 🟢 已完成

| 项目 | 说明 |
|------|------|
| 裁判计分系统 | 射击命中、到达区域、障碍通过、终点停靠计分，成绩单生成 |
| 1 号固定靶（环形）检测 | 霍夫圆变换 + 环数计分 |
| AR 标签识别 | ar_track_alvar 位姿估计 + 像素投影 |
| 识别任务板生成 | 裁判系统随机选图、生成 SDF 模型，Gazebo 中显示 |
| 导航框架 | AMCL 定位 + 全局/局部规划 + 多目标点顺序导航 |
| 场地与机器人模型 | race.world + abot URDF（麦克纳姆轮/阿克曼） |
| SLAM 建图 | Gmapping + Cartographer 均已配置 |

---

## 快速开始

### 环境要求

- Ubuntu 18.04
- ROS Melodic
- Gazebo 9
- OpenCV 3.x
- catkin tools

### 编译

```bash
cd ~/shoot_race
catkin_make
source devel/setup.bash
```

### 运行

**1. 启动仿真环境（Gazebo + 机器人模型）：**
```bash
roslaunch abot_model race.launch   # 具体 launch 文件名以实际为准
```

**2. 启动导航：**
```bash
roslaunch robot_slam navigation.launch
```

**3. 启动目标检测：**
```bash
roslaunch target_detector target_detector.launch
roslaunch target_detector alvar_detection.launch
```

**4. 启动裁判系统：**
```bash
roslaunch referee_system referee.launch
```

**5. 启动射击仿真：**
```bash
roslaunch shoot_sim_race shoot_targets.launch
```

**6. 比赛开始（触发裁判系统计时）：**
```bash
rostopic pub /shoot_race/spawn_task_board std_msgs/String "data: 'camouflage,gray,3'"
# 格式: <2号靶位置>,<3号靶位置>,<旋转靶目标叶片1~5>
# 位置: camouflage(迷彩) 或 gray(灰色)
```

---

## 话题接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/shoot_targets/hit` | `std_msgs/String` | 参赛方→裁判 | 命中汇报，格式 `fixed score=N bullet=X` / `wheel leaf=N bullet=X` / `moving region=N bullet=X` |
| `/shoot_race/arrival` | `std_msgs/String` | 参赛方→裁判 | 到达点位汇报：`shoot_1` / `shoot_2` / `shoot_3` / `finish` |
| `/shoot_race/spawn_task_board` | `std_msgs/String` | 参赛方→裁判 | 比赛开始，格式 `<2号位>,<3号位>,<旋转靶叶片>` |
| `~score` | `std_msgs/Int32` | 裁判→参赛方 | 实时累计总分（latch） |
| `/target_center_pixel` | `geometry_msgs/PointStamped` | target_detector | 靶心像素坐标 |
| `/ar_marker_pixels` | `target_detector/MarkerPixelArray` | target_detector | AR 码像素坐标 |

---

## 参赛方待实现功能清单

1. **大模型视觉识别**：对任务板拍照，调用 VLM API（如 Claude/GPT-4V）识别军火道具图片，判断应射击的阵地区域
2. **射击决策与控制**：到达射击点后，根据靶子检测结果计算瞄准角度并触发射击
3. **障碍 A 避障策略**：利用激光雷达/深度相机感知障碍物，规划无碰路径
4. **障碍 B 越障动作**：控制机器人越过障碍物
5. **完整比赛状态机**：编排从起点到终点的全流程自动执行
6. **技术文档/答辩**：撰写技术方案文档或准备答辩（10 分）
