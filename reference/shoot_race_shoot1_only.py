#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
射击比赛仿真 - 完整流程：三个射击点、越障与终点停靠

流程：
  输入靶子配置 → 等待模型加载
  → 导航至 shoot_1 → PID 瞄准环形靶 → 射击 → 上报到达
  → 导航至 shoot_2 → 追踪指定旋转叶片 → 射击 → 上报到达
  → 导航至 shoot_3 → 面向右侧任务图片墙，通过 Moonshot 视觉模型识别
  → 根据识别结果选择移动靶区域并射击
  → 穿过障碍 A → 直行越过障碍 B → 导航至终点并上报

用法：
  1. 启动仿真：roslaunch abot_model gazebo_world_2026.launch（或 launch_race.sh）
  2. 启动导航：roslaunch robot_slam navigation.launch
  3. 启动环形靶检测：roslaunch target_detector target_detector.launch
  4. 启动旋转靶标签检测：roslaunch target_detector alvar_detection.launch
  5. 运行本脚本：python3 shoot_race_shoot1_only.py
"""

import actionlib
import rospy
import time
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Twist
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from target_detector.msg import MarkerPixelArray
from math import atan2, cos, pi, radians, sin

from moonshot_vision import MoonshotVisionError, MoonshotVisionRecognizer
from race_params import (
    DIRECT_DRIVE_SPEED,
    DIRECT_DRIVE_TIMEOUT,
    DIRECT_X_KP,
    DIRECT_X_MAX,
    DIRECT_YAW_KP,
    DIRECT_YAW_MAX,
    DIRECT_YAW_MIN,
    DIRECT_YAW_TOLERANCE,
    FINE_ANGULAR_KP,
    FINE_ANGULAR_MAX,
    FINE_ANGULAR_MIN,
    FINE_CONTROL_TIMEOUT,
    FINE_LINEAR_KP,
    FINE_LINEAR_MAX,
    FINE_POSITION_TOLERANCE,
    FINE_YAW_TOLERANCE,
    FINISH_GOAL,
    MOONSHOT_MIN_CONFIDENCE,
    MOONSHOT_REQUEST_RETRIES,
    MOONSHOT_REQUEST_TIMEOUT,
    MOVING_TARGET_MODEL,
    NARROW_PASSAGE_GOALS,
    NAV_TIMEOUT,
    OBSTACLE_B_EXIT_X,
    OBSTACLE_B_TARGET_Y,
    POINT_DWELL_TIME,
    SHOOT1_AIM_TIMEOUT,
    SHOOT1_ANGULAR_KP,
    SHOOT1_ANGULAR_MAX,
    SHOOT1_HORIZONTAL_TOLERANCE,
    SHOOT2_AIM_TIMEOUT,
    SHOOT2_ANGULAR_KP,
    SHOOT2_ANGULAR_MAX,
    SHOOT2_HORIZONTAL_TOLERANCE,
    SHOOT2_MARKER_MAX_AGE,
    SHOOT2_STABLE_FRAMES,
    SHOOT2_VERTICAL_TOLERANCE,
    SHOOT3_AIM_TIMEOUT,
    SHOOT3_REGION_X_OFFSET,
    SHOOT3_STABLE_FRAMES,
    SHOOT3_TRACK_KP,
    SHOOT3_TRACK_MAX,
    SHOOT3_X_TOLERANCE,
    SHOOT_1_GOAL,
    SHOOT_2_GOAL,
    SHOOT_3_INSPECTION_GOAL,
    SHOOT_3_SHOOT_GOAL,
    SHOOT_SERVICE,
    TARGET_MODEL_NAMES,
    TASK_BOARD_FALLBACK_REGION,
    TASK_BOARD_IMAGE_MAX_AGE,
    TASK_BOARD_IMAGE_TOPIC,
    TASK_BOARD_IMAGE_WAIT_TIMEOUT,
)


def euler_to_quaternion(yaw_deg):
    yaw = radians(yaw_deg)
    q = Quaternion()
    q.x, q.y = 0, 0
    q.z = sin(yaw / 2)
    q.w = cos(yaw / 2)
    return q


def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def angular_command(error, kp, min_speed, max_speed):
    """带底盘转向死区补偿的角速度 P 控制。"""
    command = max(-max_speed, min(max_speed, kp * error))
    if 0 < abs(command) < min_speed:
        command = min_speed if command > 0 else -min_speed
    return command


class Shoot1Only:
    def __init__(self):
        rospy.init_node("shoot_race_shoot1_only", anonymous=False)

        self.arrival_pub = rospy.Publisher("/shoot_race/arrival", String, queue_size=1)
        self.spawn_pub = rospy.Publisher("/shoot_race/spawn_task_board", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self._model_states = None
        self._target_center = None
        self._marker_pixels = {}
        self._marker_stamp = None
        self._camera_center_u = 400
        self._camera_center_v = 400
        self._shoot_line_v = None
        self._bridge = CvBridge()
        self._latest_camera_image = None
        self._latest_camera_stamp = None
        self._vision_recognizer = MoonshotVisionRecognizer(
            timeout=MOONSHOT_REQUEST_TIMEOUT,
            retries=MOONSHOT_REQUEST_RETRIES,
            min_confidence=MOONSHOT_MIN_CONFIDENCE,
        )

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)
        rospy.Subscriber("/target_center_pixel", PointStamped, self._on_target_center, queue_size=1)
        rospy.Subscriber("/ar_marker_pixels", MarkerPixelArray, self._on_marker_pixels, queue_size=1)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber(TASK_BOARD_IMAGE_TOPIC, Image, self._on_camera_image, queue_size=1)

    def _on_model_states(self, msg):
        self._model_states = msg

    def _on_target_center(self, msg):
        self._target_center = (msg.point.x, msg.point.y)

    def _on_marker_pixels(self, msg):
        self._marker_pixels = {marker.id: (marker.u, marker.v) for marker in msg.markers}
        self._marker_stamp = rospy.Time.now()

    def _on_camera_info(self, msg):
        if len(msg.K) >= 6:
            self._camera_center_u = msg.K[2]
            self._camera_center_v = msg.K[5]

    def _on_camera_image(self, msg):
        try:
            self._latest_camera_image = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            ).copy()
            self._latest_camera_stamp = rospy.Time.now()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[任务识别] 相机图像转换失败: %s", exc)

    def wait_for_move_base(self, timeout=60):
        rospy.loginfo("[shoot1] 等待 move_base...")
        if not self.move_base.wait_for_server(rospy.Duration(timeout)):
            rospy.logerr("[shoot1] move_base 未就绪")
            return False
        rospy.loginfo("[shoot1] move_base 已就绪")
        return True

    def send_spawn_task_board(self, msg_str):
        rospy.loginfo("[shoot1] 发布比赛开始: %s", msg_str)
        self.spawn_pub.publish(String(data=msg_str))
        rospy.sleep(0.5)

    def wait_for_target_models(self, timeout=30):
        rospy.loginfo("[shoot1] 等待靶子模型加载...")
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("[shoot1] 等待模型超时")
                return False
            if self._model_states is not None:
                names = set(self._model_states.name)
                if all(n in names for n in TARGET_MODEL_NAMES):
                    rospy.loginfo("[shoot1] 3 个靶子模型已加载完成")
                    return True
            rate.sleep()
        return False

    def send_arrival(self, label):
        rospy.loginfo("[shoot1] 上报到达: %s", label)
        self.arrival_pub.publish(String(data=label))
        rospy.sleep(0.5)

    def _stop_robot(self):
        t = Twist()
        t.linear.x = t.linear.y = t.linear.z = 0
        t.angular.x = t.angular.y = t.angular.z = 0
        self.cmd_vel_pub.publish(t)

    def _robot_xy_yaw(self):
        """从 Gazebo 获取机器人真实平面位姿。"""
        if self._model_states is None:
            return None
        try:
            index = self._model_states.name.index("abot_model")
            pose = self._model_states.pose[index]
        except (ValueError, IndexError):
            return None
        q = pose.orientation
        yaw = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return pose.position.x, pose.position.y, yaw

    def _model_xy(self, model_name):
        """读取指定 Gazebo 模型的世界坐标。"""
        if self._model_states is None:
            return None
        try:
            index = self._model_states.name.index(model_name)
            pose = self._model_states.pose[index]
        except (ValueError, IndexError):
            return None
        return pose.position.x, pose.position.y

    def _pid_angular(self, error_u, kp, max_speed):
        out = kp * error_u
        out = max(-max_speed, min(max_speed, out))
        return -out

    def _do_shoot(self):
        try:
            shoot_srv = rospy.ServiceProxy(SHOOT_SERVICE, Trigger)
            resp = shoot_srv()
            return resp.success if resp else False
        except Exception as e:
            rospy.logerr("[射击] 服务调用失败: %s", e)
            return False

    def shoot_at_ring_target(self):
        """环形靶：横向 PID 对准后射击"""
        rospy.loginfo("[shoot1] 瞄准环形靶...")
        self._target_center = None
        rate = rospy.Rate(20)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > SHOOT1_AIM_TIMEOUT:
                rospy.logwarn("[shoot1] 瞄准超时")
                self._stop_robot()
                return False
            if self._target_center is None:
                rospy.loginfo_throttle(2.0, "[shoot1] 等待靶心检测（请确认 target_detector 已启动）")
                self._stop_robot()
                rate.sleep()
                continue
            u, v = self._target_center
            error_u = u - self._camera_center_u
            ang = self._pid_angular(
                error_u,
                SHOOT1_ANGULAR_KP,
                SHOOT1_ANGULAR_MAX,
            )
            if abs(error_u) < SHOOT1_HORIZONTAL_TOLERANCE:
                self._stop_robot()
                # 固定靶靶心与弹道同高，记录其像素行作为后续旋转靶的枪口高度线。
                self._shoot_line_v = v
                rospy.sleep(0.2)
                if self._do_shoot():
                    rospy.loginfo("[shoot1] 射击完成，标定弹道像素行 v=%.1f", v)
                    return True
                rospy.sleep(0.5)
                continue
            t = Twist()
            t.angular.z = ang
            self.cmd_vel_pub.publish(t)
            rate.sleep()
        return False

    def shoot_at_wheel_target(self, target_marker_id):
        """旋转靶：追踪指定 ALVAR 标签，横向对准并等待它经过枪口高度。"""
        rospy.loginfo("[shoot2] 瞄准旋转靶指定叶片 %d...", target_marker_id)
        self._marker_pixels = {}
        self._marker_stamp = None
        stable_frames = 0
        rate = rospy.Rate(30)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - start).to_sec() > SHOOT2_AIM_TIMEOUT:
                rospy.logwarn("[shoot2] 瞄准超时，未等到 %d 号叶片进入射击窗口", target_marker_id)
                self._stop_robot()
                return False

            marker_is_fresh = (
                self._marker_stamp is not None
                and (now - self._marker_stamp).to_sec() <= SHOOT2_MARKER_MAX_AGE
            )
            marker_pixel = self._marker_pixels.get(target_marker_id) if marker_is_fresh else None
            if marker_pixel is None:
                stable_frames = 0
                rospy.loginfo_throttle(
                    2.0,
                    "[shoot2] 等待 %d 号叶片（请确认 alvar_detection.launch 已启动）",
                    target_marker_id,
                )
                self._stop_robot()
                rate.sleep()
                continue

            u, v = marker_pixel
            error_u = u - self._camera_center_u
            shoot_line_v = (
                self._shoot_line_v
                if self._shoot_line_v is not None
                else self._camera_center_v
            )
            error_v = v - shoot_line_v

            # 车体只能修正水平方向；叶片高度随旋转变化，因此纵向只等待合适时机。
            if abs(error_u) >= SHOOT2_HORIZONTAL_TOLERANCE:
                stable_frames = 0
                t = Twist()
                t.angular.z = self._pid_angular(
                    error_u,
                    SHOOT2_ANGULAR_KP,
                    SHOOT2_ANGULAR_MAX,
                )
                self.cmd_vel_pub.publish(t)
                rate.sleep()
                continue

            self._stop_robot()
            if abs(error_v) <= SHOOT2_VERTICAL_TOLERANCE:
                stable_frames += 1
                if stable_frames >= SHOOT2_STABLE_FRAMES:
                    rospy.loginfo(
                        "[shoot2] %d 号叶片进入射击窗口，像素误差=(%.1f, %.1f)",
                        target_marker_id,
                        error_u,
                        error_v,
                    )
                    if self._do_shoot():
                        rospy.loginfo("[shoot2] 射击完成")
                        return True
                    stable_frames = 0
                    rospy.sleep(0.5)
            else:
                stable_frames = 0
                rospy.loginfo_throttle(
                    1.0,
                    "[shoot2] 等待 %d 号叶片经过枪口高度，纵向误差 %.1f px",
                    target_marker_id,
                    error_v,
                )
            rate.sleep()
        return False

    def recognize_task_board(self):
        """拍摄任务板并通过 Moonshot 视觉模型返回目标区域。"""
        rospy.loginfo("[任务识别] 等待任务板相机画面: %s", TASK_BOARD_IMAGE_TOPIC)
        start_wall = time.monotonic()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            image_ready = self._latest_camera_image is not None
            image_fresh = (
                self._latest_camera_stamp is not None
                and (rospy.Time.now() - self._latest_camera_stamp).to_sec()
                <= TASK_BOARD_IMAGE_MAX_AGE
            )
            if image_ready and image_fresh:
                break
            if time.monotonic() - start_wall > TASK_BOARD_IMAGE_WAIT_TIMEOUT:
                rospy.logwarn(
                    "[任务识别] 等待相机超时，回退区域 %d",
                    TASK_BOARD_FALLBACK_REGION,
                )
                return TASK_BOARD_FALLBACK_REGION
            rate.sleep()

        try:
            result = self._vision_recognizer.recognize(
                self._latest_camera_image.copy()
            )
            rospy.loginfo(
                "[任务识别] object=%s confidence=%.2f region=%d description=%s",
                result.object_name,
                result.confidence,
                result.region,
                result.description,
            )
            return result.region
        except MoonshotVisionError as exc:
            rospy.logwarn(
                "[任务识别] %s，回退区域 %d",
                exc,
                TASK_BOARD_FALLBACK_REGION,
            )
            return TASK_BOARD_FALLBACK_REGION

    def shoot_at_moving_target(self, target_region):
        """跟踪移动靶，并根据视觉识别结果射击区域 1/2/3。"""
        if target_region not in SHOOT3_REGION_X_OFFSET:
            rospy.logwarn(
                "[shoot3] 非法区域 %s，回退区域 %d",
                target_region,
                TASK_BOARD_FALLBACK_REGION,
            )
            target_region = TASK_BOARD_FALLBACK_REGION
        region_offset = SHOOT3_REGION_X_OFFSET[target_region]
        rospy.loginfo(
            "[shoot3] 选择移动靶区域=%d，横向偏移=%.3fm，开始跟踪",
            target_region,
            region_offset,
        )
        self.move_base.cancel_all_goals()
        rospy.sleep(0.2)
        target_yaw = radians(90)
        hold_y = SHOOT_3_SHOOT_GOAL[1]
        stable_frames = 0
        rate = rospy.Rate(30)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > SHOOT3_AIM_TIMEOUT:
                rospy.logwarn("[shoot3] 移动靶中心瞄准超时")
                self._stop_robot()
                return False

            robot = self._robot_xy_yaw()
            target = self._model_xy(MOVING_TARGET_MODEL)
            if robot is None or target is None:
                stable_frames = 0
                self._stop_robot()
                rospy.loginfo_throttle(2.0, "[shoot3] 等待移动靶和机器人位姿")
                rate.sleep()
                continue

            robot_x, robot_y, robot_yaw = robot
            target_x, _ = target
            aim_x = target_x + region_offset
            error_x = aim_x - robot_x
            error_y = hold_y - robot_y
            yaw_error = normalize_angle(target_yaw - robot_yaw)

            if (
                abs(error_x) <= SHOOT3_X_TOLERANCE
                and abs(error_y) <= FINE_POSITION_TOLERANCE
                and abs(yaw_error) <= FINE_YAW_TOLERANCE
            ):
                stable_frames += 1
                self._stop_robot()
                if stable_frames >= SHOOT3_STABLE_FRAMES:
                    rospy.loginfo(
                        "[shoot3] 区域%d对准，robot_x=%.3f aim_x=%.3f target_x=%.3f",
                        target_region,
                        robot_x,
                        aim_x,
                        target_x,
                    )
                    if self._do_shoot():
                        rospy.loginfo("[shoot3] 移动靶区域%d射击完成", target_region)
                        return True
                    stable_frames = 0
                    rospy.sleep(0.5)
            else:
                stable_frames = 0
                t = Twist()
                # 面向北时，局部 +y 对应世界 -x。
                t.linear.y = max(
                    -SHOOT3_TRACK_MAX,
                    min(SHOOT3_TRACK_MAX, -SHOOT3_TRACK_KP * error_x),
                )
                t.linear.x = max(
                    -FINE_LINEAR_MAX,
                    min(FINE_LINEAR_MAX, FINE_LINEAR_KP * error_y),
                )
                t.angular.z = angular_command(
                    yaw_error,
                    FINE_ANGULAR_KP,
                    FINE_ANGULAR_MIN,
                    FINE_ANGULAR_MAX,
                )
                self.cmd_vel_pub.publish(t)
                rospy.loginfo_throttle(
                    1.0,
                    "[shoot3] 跟踪区域%d x误差=%.3f y误差=%.3f yaw误差=%.1f°",
                    target_region,
                    error_x,
                    error_y,
                    yaw_error * 180.0 / pi,
                )
            rate.sleep()
        return False

    def align_to_yaw(self, yaw_deg, timeout=10):
        """使用 Gazebo 真实位姿原地摆正，为障碍 B 直行做准备。"""
        target_yaw = radians(yaw_deg)
        rate = rospy.Rate(30)
        start = rospy.Time.now()
        stable_frames = 0
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout:
                rospy.logwarn("[障碍B] 摆正朝向超时")
                self._stop_robot()
                return False
            state = self._robot_xy_yaw()
            if state is None:
                self._stop_robot()
                rate.sleep()
                continue
            _, _, yaw = state
            error = normalize_angle(target_yaw - yaw)
            if abs(error) <= DIRECT_YAW_TOLERANCE:
                stable_frames += 1
                self._stop_robot()
                if stable_frames >= 5:
                    return True
            else:
                stable_frames = 0
                t = Twist()
                t.angular.z = angular_command(
                    error,
                    DIRECT_YAW_KP,
                    DIRECT_YAW_MIN,
                    DIRECT_YAW_MAX,
                )
                self.cmd_vel_pub.publish(t)
            rate.sleep()
        return False

    def refine_pose(self, x, y, yaw_deg, timeout=FINE_CONTROL_TIMEOUT):
        """move_base 粗导航后，直接控制底盘完成厘米级位置和角度修正。"""
        self.move_base.cancel_all_goals()
        rospy.sleep(0.2)
        target_yaw = radians(yaw_deg)
        rate = rospy.Rate(30)
        start = rospy.Time.now()
        stable_frames = 0

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout:
                state = self._robot_xy_yaw()
                self._stop_robot()
                rospy.logwarn("[精对位] 超时，当前位姿=%s", state)
                return False
            state = self._robot_xy_yaw()
            if state is None:
                self._stop_robot()
                rate.sleep()
                continue

            current_x, current_y, current_yaw = state
            error_x = x - current_x
            error_y = y - current_y
            error_yaw = normalize_angle(target_yaw - current_yaw)

            if (
                abs(error_x) <= FINE_POSITION_TOLERANCE
                and abs(error_y) <= FINE_POSITION_TOLERANCE
                and abs(error_yaw) <= FINE_YAW_TOLERANCE
            ):
                stable_frames += 1
                self._stop_robot()
                if stable_frames >= 5:
                    rospy.loginfo(
                        "[精对位] 完成，实际=(%.3f, %.3f, %.1f°)",
                        current_x,
                        current_y,
                        current_yaw * 180.0 / pi,
                    )
                    return True
            else:
                stable_frames = 0
                # 将世界坐标误差转换到机器人局部坐标，再发布麦轮速度。
                local_x = cos(current_yaw) * error_x + sin(current_yaw) * error_y
                local_y = -sin(current_yaw) * error_x + cos(current_yaw) * error_y
                t = Twist()
                t.linear.x = max(
                    -FINE_LINEAR_MAX,
                    min(FINE_LINEAR_MAX, FINE_LINEAR_KP * local_x),
                )
                t.linear.y = max(
                    -FINE_LINEAR_MAX,
                    min(FINE_LINEAR_MAX, FINE_LINEAR_KP * local_y),
                )
                t.angular.z = angular_command(
                    error_yaw,
                    FINE_ANGULAR_KP,
                    FINE_ANGULAR_MIN,
                    FINE_ANGULAR_MAX,
                )
                self.cmd_vel_pub.publish(t)
                rospy.loginfo_throttle(
                    1.0,
                    "[精对位] 误差 x=%.3f y=%.3f yaw=%.1f°",
                    error_x,
                    error_y,
                    error_yaw * 180.0 / pi,
                )
            rate.sleep()
        return False

    def cross_obstacle_b(self):
        """从东向西持续直行，越过障碍 B 后进入终点一侧。"""
        # 防止 move_base 与本函数同时向 /cmd_vel 发布互相覆盖的速度。
        self.move_base.cancel_all_goals()
        rospy.sleep(0.5)
        rospy.loginfo("[障碍B] 摆正至 180°，准备从东向西越障")
        if not self.align_to_yaw(180):
            return False

        start_state = self._robot_xy_yaw()
        if start_state is None:
            rospy.logwarn("[障碍B] 无法获取机器人位姿")
            return False
        target_y = OBSTACLE_B_TARGET_Y
        target_yaw = radians(180)
        rate = rospy.Rate(30)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > DIRECT_DRIVE_TIMEOUT:
                rospy.logwarn("[障碍B] 直接越障超时")
                self._stop_robot()
                return False
            state = self._robot_xy_yaw()
            if state is None:
                self._stop_robot()
                rate.sleep()
                continue
            x, y, yaw = state
            if x <= OBSTACLE_B_EXIT_X:
                self._stop_robot()
                rospy.sleep(0.5)
                rospy.loginfo("[障碍B] 已越过横向隔断，当前位置 (%.2f, %.2f)", x, y)
                return True

            yaw_error = normalize_angle(target_yaw - yaw)
            world_y_error = target_y - y
            t = Twist()
            t.linear.x = DIRECT_DRIVE_SPEED
            # 机器人面向西时，局部 +y 对应世界 -y，因此取反修正世界 y。
            t.linear.y = max(
                -DIRECT_X_MAX,
                min(DIRECT_X_MAX, -DIRECT_X_KP * world_y_error),
            )
            t.angular.z = angular_command(
                yaw_error,
                DIRECT_YAW_KP,
                DIRECT_YAW_MIN,
                DIRECT_YAW_MAX,
            )
            self.cmd_vel_pub.publish(t)
            rospy.loginfo_throttle(
                1.0,
                "[障碍B] 越障中 position=(%.2f, %.2f) yaw=%.1f° cmd=(%.2f, %.2f, %.2f)",
                x,
                y,
                yaw * 180.0 / pi,
                t.linear.x,
                t.linear.y,
                t.angular.z,
            )
            rate.sleep()
        return False

    def goto_goal(self, x, y, yaw_deg, fine_align=False):
        """通过 actionlib 发送导航目标，确保结果属于本次目标。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = euler_to_quaternion(yaw_deg)
        rospy.loginfo("[导航] 发布目标 (%.2f, %.2f) yaw=%.0f°", x, y, yaw_deg)
        self.move_base.send_goal(goal)
        if not self.move_base.wait_for_result(rospy.Duration(NAV_TIMEOUT)):
            self.move_base.cancel_goal()
            self._stop_robot()
            rospy.logwarn("[导航] 超时")
            return False
        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            self._stop_robot()
            rospy.logwarn("[导航] move_base 返回 %s", state)
            return False

        actual = self._robot_xy_yaw()
        if actual is not None:
            rospy.loginfo(
                "[导航] move_base 成功，实际=(%.3f, %.3f, %.1f°)",
                actual[0],
                actual[1],
                actual[2] * 180.0 / pi,
            )
        if fine_align:
            return self.refine_pose(x, y, yaw_deg)
        return True

    def run(self, wheel_target_id):
        if not self.wait_for_move_base():
            return

        x, y, yaw = SHOOT_1_GOAL
        rospy.loginfo("[shoot1] 前往 shoot_1 (%.2f, %.2f)", x, y)
        if not self.goto_goal(x, y, yaw, fine_align=True):
            rospy.logwarn("[shoot1] 未到达 shoot_1")
            return
        self.shoot_at_ring_target()
        self.send_arrival("shoot_1")

        x, y, yaw = SHOOT_2_GOAL
        rospy.loginfo("[shoot2] 前往 shoot_2 (%.3f, %.3f)", x, y)
        if not self.goto_goal(x, y, yaw, fine_align=True):
            rospy.logwarn("[shoot2] 未到达 shoot_2")
            return
        if not self.shoot_at_wheel_target(wheel_target_id):
            rospy.logwarn("[shoot2] 旋转靶射击未完成，继续后续流程")
        self.send_arrival("shoot_2")

        x, y, yaw = SHOOT_3_INSPECTION_GOAL
        rospy.loginfo("[shoot3] 前往 shoot_3，先面向右侧任务图片墙")
        if not self.goto_goal(x, y, yaw, fine_align=True):
            rospy.logwarn("[shoot3] 未到达 shoot_3")
            return
        self._stop_robot()
        self.send_arrival("shoot_3")
        rospy.loginfo("[shoot3] 面向任务图片墙，等待画面稳定 %.1f 秒", POINT_DWELL_TIME)
        rospy.sleep(POINT_DWELL_TIME)
        target_region = self.recognize_task_board()

        x, y, yaw = SHOOT_3_SHOOT_GOAL
        rospy.loginfo("[shoot3] 转向移动靶并重新精对位")
        if not self.refine_pose(x, y, yaw):
            rospy.logwarn("[shoot3] 转向移动靶失败")
            return
        if not self.shoot_at_moving_target(target_region):
            rospy.logwarn("[shoot3] 移动靶区域%d射击失败，继续后续流程", target_region)

        rospy.loginfo("[路线] 障碍 A 已在前往 shoot_3 的途中自动通过，不再返回")
        rospy.loginfo("[狭窄通道] 朝 3 号靶方向向北，再前往障碍 B 东侧入口")
        for index, goal in enumerate(NARROW_PASSAGE_GOALS, start=1):
            fine_align = index == len(NARROW_PASSAGE_GOALS)
            if not self.goto_goal(*goal, fine_align=fine_align):
                rospy.logwarn("[狭窄通道] 第 %d 个航点失败: %s", index, goal)
                return
        if not self.cross_obstacle_b():
            rospy.logwarn("[障碍B] 越障失败")
            return

        x, y, yaw = FINISH_GOAL
        rospy.loginfo("[终点] 前往终点停靠位 (%.2f, %.2f)", x, y)
        if not self.goto_goal(x, y, yaw, fine_align=True):
            rospy.logwarn("[终点] 未到达终点停靠位")
            return
        self._stop_robot()
        rospy.sleep(POINT_DWELL_TIME)
        self.send_arrival("finish")
        rospy.loginfo("[全流程] 已到达终点，可查看 /referee/score 和成绩单")


def parse_wheel_target(spawn_msg):
    """读取 target2,target3,wheel_target 中的指定旋转靶叶片。"""
    parts = [part.strip() for part in spawn_msg.split(",")]
    if len(parts) < 3:
        print("[配置] 未提供 wheel_target，默认使用 1 号叶片")
        return 1
    try:
        target_id = int(parts[2])
    except ValueError:
        print("[配置] wheel_target={!r} 无效，默认使用 1 号叶片".format(parts[2]))
        return 1
    if target_id not in range(1, 6):
        print("[配置] wheel_target 必须为 1~5，默认使用 1 号叶片")
        return 1
    return target_id


def main():
    print("=" * 50)
    print("射击比赛仿真 - 完整流程（shoot_1 + shoot_2 + shoot_3 + 障碍 + 终点）")
    print("=" * 50)
    print("请输入靶子配置（格式 target2,target3,wheel_target）：")
    print("  例如: c,g,3")
    print("-" * 50)
    spawn_msg = input(">>> ").strip() or "c,g,3"
    wheel_target_id = parse_wheel_target(spawn_msg)
    print("旋转靶指定叶片: {}".format(wheel_target_id))
    print("-" * 50)

    ref = Shoot1Only()
    rospy.sleep(2)

    ref.send_spawn_task_board(spawn_msg)
    if not ref.wait_for_target_models():
        rospy.logwarn("[shoot1] 继续执行")

    print("")
    print("目标已设置，模型已加载。")
    input("按回车键开始运行 >>> ")
    print("")

    ref.run(wheel_target_id)


if __name__ == "__main__":
    main()
