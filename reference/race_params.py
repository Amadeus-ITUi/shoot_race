#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""射击比赛主流程集中调参文件。

除角度容差特别注明为“度”外，程序内部角度单位使用弧度。
位置单位为米，速度单位为 m/s 或 rad/s，图像容差单位为像素。
"""

from math import radians


# =============================================================================
# 导航点与流程
# =============================================================================

# (世界坐标 x, 世界坐标 y, 朝向角度)
SHOOT_1_GOAL = (1.80, 0.20, 0)
SHOOT_2_GOAL = (1.16, 1.125, 180)
SHOOT_3_INSPECTION_GOAL = (2.68, 1.88, 0)
SHOOT_3_SHOOT_GOAL = (2.68, 1.88, 90)

NARROW_PASSAGE_GOALS = [
    (2.68, 2.30, 90),
    (2.68, 2.75, 90),
    (2.15, 2.925, 180),
]

OBSTACLE_B_TARGET_Y = 2.925
OBSTACLE_B_EXIT_X = 0.75
FINISH_GOAL = (0.02, 3.12, 180)

TARGET_MODEL_NAMES = ["target_fixed", "target_wheel", "target_moving"]
NAV_TIMEOUT = 120
POINT_DWELL_TIME = 1.0


# =============================================================================
# shoot1：固定环形靶
# =============================================================================

# 像素横向误差到车体角速度的比例控制。
SHOOT1_ANGULAR_KP = 5.5
SHOOT1_ANGULAR_MAX = 0.5
SHOOT1_HORIZONTAL_TOLERANCE = 15
SHOOT1_AIM_TIMEOUT = 30
SHOOT_SERVICE = "/shoot_sim/shoot"


# =============================================================================
# shoot2：旋转靶
# =============================================================================

# 实机采用单次射击，不监听仿真命中反馈重复开枪。
SHOOT2_ANGULAR_KP = 5.5
SHOOT2_ANGULAR_MAX = 0.5
SHOOT2_HORIZONTAL_TOLERANCE = 12
SHOOT2_VERTICAL_TOLERANCE = 12
SHOOT2_MARKER_MAX_AGE = 0.5
SHOOT2_STABLE_FRAMES = 2
SHOOT2_AIM_TIMEOUT = 30


# =============================================================================
# shoot3：移动靶
# =============================================================================

MOVING_TARGET_MODEL = "target_moving"
SHOOT3_AIM_TIMEOUT = 15
SHOOT3_X_TOLERANCE = 0.015
SHOOT3_STABLE_FRAMES = 4
SHOOT3_TRACK_KP = 1.4
SHOOT3_TRACK_MAX = 0.12

# 目标板宽约 0.18m。机器人对准移动靶中心左/中/右侧来射击区域 1/2/3。
SHOOT3_REGION_X_OFFSET = {
    1: -0.12,
    2: 0.0,
    3: 0.12,
}


# =============================================================================
# 任务板 Moonshot 视觉识别
# =============================================================================

TASK_BOARD_IMAGE_TOPIC = "/camera/image"
TASK_BOARD_IMAGE_MAX_AGE = 2.0
TASK_BOARD_IMAGE_WAIT_TIMEOUT = 5.0
TASK_BOARD_FALLBACK_REGION = 2

MOONSHOT_REQUEST_TIMEOUT = 30.0
MOONSHOT_REQUEST_RETRIES = 3
MOONSHOT_MIN_CONFIDENCE = 0.55


# =============================================================================
# move_base 后的厘米级精对位
# =============================================================================

FINE_POSITION_TOLERANCE = 0.010
FINE_YAW_TOLERANCE = radians(3)
FINE_CONTROL_TIMEOUT = 15

FINE_LINEAR_KP = 1.2
FINE_LINEAR_MAX = 0.12

# 仿真底盘存在转向死区，因此保留最小角速度补偿。
FINE_ANGULAR_KP = 1.8
FINE_ANGULAR_MIN = 1.0
FINE_ANGULAR_MAX = 1.5


# =============================================================================
# 障碍 B 直接控制
# =============================================================================

DIRECT_DRIVE_TIMEOUT = 20
DIRECT_DRIVE_SPEED = 0.55

DIRECT_YAW_KP = 1.8
DIRECT_YAW_MIN = 1.0
DIRECT_YAW_MAX = 1.5
DIRECT_YAW_TOLERANCE = radians(3)

DIRECT_X_KP = 1.2
DIRECT_X_MAX = 0.15
