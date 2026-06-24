# 裁判系统话题与使用方法

裁判系统订阅的话题、发布的话题、服务，以及终端发布示例。

---

## 参赛者需要发布的话题

| 话题 | 类型 | 何时发布 | 说明 |
|------|------|----------|------|
| `/shoot_race/spawn_task_board` | `std_msgs/String` | 比赛开始时 | 格式 `target2,target3,wheel_target`，如 `c,g,3`。触发任务板生成、靶子生成；3 个靶子模型加载完成后开始计时 |
| `/shoot_race/arrival` | `std_msgs/String` | 到达任务点或终点时 | 取值 `shoot_1`、`shoot_2`、`shoot_3`、`finish` |

**障碍区域 A/B** 由裁判按机器人位姿自动判定，无需参赛者发布。

---

## 1. 到达点位 `/shoot_race/arrival`

参赛方在机器人到达对应任务点或终点时，向裁判发送到达消息，裁判据此计分。

| 项目 | 说明 |
|------|------|
| **话题** | `/shoot_race/arrival` |
| **类型** | `std_msgs/String` |
| **方向** | 参赛方 → 裁判（发布） |

### 消息取值

| 取值 | 含义 | 分值 |
|------|------|------|
| `shoot_1` | 到达射击任务点 1 | 10（在区域内） |
| `shoot_2` | 到达射击任务点 2 | 10（在区域内） |
| `shoot_3` | 到达射击任务点 3 | 10（在区域内） |
| `finish` | 到达终点 | 5 或 10（由裁判按机器人 Gazebo 位姿判定） |

### 判定规则

参赛方发送到达消息后，裁判订阅 `/gazebo/model_states` 获取机器人真实位姿，验证机器人中心是否在对应区域内：
- **射击任务点**：按机器人四轮形成的矩形与任务点框（50×35cm）的重叠程度给分。完全进入 10 分，部分进入按比例取整（0~10 整数）
- **终点**：在 10 分区域（内圈）→ 10 分；在 5 分区域（外圈）→ 5 分；区域外 → 0 分

### 终端发布示例

```bash
# 到达射击任务点 1
rostopic pub /shoot_race/arrival std_msgs/String "data: 'shoot_1'"

# 到达射击任务点 2
rostopic pub /shoot_race/arrival std_msgs/String "data: 'shoot_2'"

# 到达射击任务点 3
rostopic pub /shoot_race/arrival std_msgs/String "data: 'shoot_3'"

# 到达终点
rostopic pub /shoot_race/arrival std_msgs/String "data: 'finish'"
```

### Python 示例

```python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher("/shoot_race/arrival", String, queue_size=1)
pub.publish(String(data="shoot_1"))
pub.publish(String(data="finish"))
```

### 规则

- 每个点位**仅计分一次**，重复发送同一点位会被忽略
- 调用 `/referee/reset` 重置比赛时，会清空已到达记录
- 取值不区分大小写（`shoot_1` 与 `Shoot_1` 等效）

### 障碍区域 A/B（自动判定）

障碍区域由裁判按机器人 Gazebo 位姿**自动判定**，无需发送 arrival 消息。

| 区域 | 计分规则 |
|------|----------|
| **A（避障）** | 矩形平分为前后两半，经过前半无碰撞 +5 分，经过后半无碰撞 +5 分，共 10 分 |
| **B（越障）** | 机器人进入区域后，投影完全离开该区域才得 10 分 |

**区域 A 碰撞检测**：订阅机器人 bumper 话题（`gazebo_msgs/ContactsState`），检测底盘除底面接触外的碰撞。接触法向垂直（底面）则忽略，水平（墙面）则视为碰撞。bumper 话题由 `ob_a_bumper_topic` 配置，默认 `/abot_bumper`。

**参数（referee.launch）**：`ob_a_x`, `ob_a_y`, `ob_a_half_w`, `ob_a_half_h`, `ob_a_split_axis`, `ob_a_bumper_topic`, `ob_b_x`, `ob_b_y`, `ob_b_half_w`, `ob_b_half_h`

### 分数参数（可自定义各项目最高分）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `score_shoot_point_max` | 10 | 射击任务点 1/2/3 每个最高分 |
| `score_finish_5_max` | 5 | 终点外圈最高分 |
| `score_finish_10_max` | 5 | 终点内圈最高分 |
| `score_ob_a_half_max` | 5 | 障碍 A 每半最高分 |
| `score_ob_b_max` | 10 | 障碍 B 最高分 |
| `score_fixed_max` | 10 | 固定靶最高分 |
| `score_wheel_max` | 10 | 旋转靶最高分 |
| `score_moving_max` | 10 | 平移靶最高分 |

---

## 2. 比赛开始 `/shoot_race/spawn_task_board`

**参赛者在比赛开始时发布**，消息内容为三个靶子的位置选择。裁判会：① 随机选图生成识别任务板；② shoot_targets 按消息生成靶子；③ **3 个靶子模型在 Gazebo 中加载完成后开始计时**。

| 项目 | 说明 |
|------|------|
| **话题** | `/shoot_race/spawn_task_board` |
| **类型** | `std_msgs/String` |
| **方向** | 参赛方 → 裁判、shoot_targets（发布） |

### 消息格式

`target2,target3,wheel_target`：三个参数，逗号分隔。**1号固定靶位置固定、无系数，不在此消息中**。**平移靶目标由任务板随机图片决定**。

| 参数 | 取值 | 含义 |
|------|------|------|
| target2 | `c`/`g` | 2号旋转靶位置：c=迷彩(系数×1)，g=灰色(系数×2) |
| target3 | `c`/`g` | 3号平移靶位置：c=迷彩(系数×1)，g=灰色(系数×2) |
| wheel_target | `1`~`5` | 旋转靶 5 个小正方形，仅打中此叶片得 10×系数，否则 0 |

**1号固定靶**：位置固定，环数(6~10)计分，最高 10 分，无系数。

**平移靶目标（由任务板图片决定）：**
- `ak47.jpg`、`helmet.jpg`、`pack.jpg` → 移动靶 1 区
- `aid.jpg`、`gauze.jpg`、`iv.jpg` → 移动靶 2 区
- `mag.jpg`、`box.jpg`、`belt.jpg` → 移动靶 3 区

**计分规则：**
- 1号固定靶：环数(6~10)，最高 10 分，无系数，位置固定
- 2号旋转靶：打中指定叶片得 10×系数，打中其他叶片得 0
- 3号平移靶：打中指定区域得 10×系数，打中其他区域得 0（目标由任务板图片决定）
- **射击靶子只计第一次命中**：同一靶子多次命中仅第一次得分，后续忽略

### 终端发布示例

```bash
# 2号迷彩、3号灰，旋转靶打 3 号叶片（平移靶目标由随机选图决定）
rostopic pub /shoot_race/spawn_task_board std_msgs/String "data: 'c,g,3'"

# 全迷彩，旋转靶打 1 号叶片
rostopic pub /shoot_race/spawn_task_board std_msgs/String "data: 'c,c,1'"

# 仅 2 个参数（向后兼容）：位置 c,g，wheel_target=1
rostopic pub /shoot_race/spawn_task_board std_msgs/String "data: 'c,g'"
```

### Python 示例

```python
from std_msgs.msg import String

pub = rospy.Publisher("/shoot_race/spawn_task_board", String, queue_size=1)
# 2号迷彩、3号灰，旋转靶打3号叶片（平移靶目标由任务板随机图片决定）
pub.publish(String(data="c,g,3"))
```

### 说明

- 发布后：① 生成 A4 竖版识别任务板（随机选图）；② shoot_targets 按消息生成靶子；③ 3 个靶子模型加载完成后开始计时
- 收到 `/shoot_race/arrival` 的 `finish` 时，裁判在终端打印总分和用时
- 同分时按用时排名，用时少者靠前

### 参数（referee.launch 可调）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `task_board_x` | 2.0 | 任务板中心 X (m) |
| `task_board_y` | 1.0 | 任务板中心 Y (m) |
| `task_board_z` | 0.5 | 任务板中心 Z (m) |
| `task_board_yaw` | 0.0 | 朝向 (rad) |
| `task_board_width` | 0.21 | 板面宽 (m)，A4 竖版 210mm |
| `task_board_height` | 0.297 | 板面高 (m)，A4 竖版 297mm |

### 图片

任务板图片来自 `referee_system/images/`，支持 `.jpg`、`.jpeg`、`.png`，每次触发随机选一张。所选图片同时决定**平移靶目标区域**：
- ak47、helmet、pack → 1 区
- aid、gauze、iv → 2 区
- mag、box、belt → 3 区

---

## 3. 得分 `/referee/score`

裁判发布的当前总分。

| 项目 | 说明 |
|------|------|
| **话题** | `/referee/score` |
| **类型** | `std_msgs/Int32` |
| **方向** | 裁判 → 外部（订阅） |

### 终端订阅示例

```bash
rostopic echo /referee/score
```

---

## 4. 重置 `/referee/reset`

重置比赛得分与已到达记录。

| 项目 | 说明 |
|------|------|
| **服务** | `/referee/reset` |
| **类型** | `std_srvs/Trigger` |

### 终端调用示例

```bash
rosservice call /referee/reset "{}"
```
