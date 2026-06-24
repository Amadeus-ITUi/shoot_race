#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
射击比赛仿真 - 简化版：仅导航到第一个射击点并完成射击

流程：输入靶子配置 → 等待模型加载 → 导航至 shoot_1 → PID 瞄准环形靶 → 射击 → 上报到达

用法：
  1. 启动仿真：roslaunch abot_model gazebo_world_2026.launch（或 launch_race.sh）
  2. 启动导航：roslaunch robot_slam navigation.launch
  3. 启动环形靶检测：roslaunch target_detector target_detector.launch
  4. 运行本脚本：python3 shoot_race_shoot1_only.py
"""

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from math import radians, sin, cos


# ============ 导航点 ============
SHOOT_1_GOAL = (1.85, 0.23, 0)   # shoot_1：射击任务点 1（环形靶）
TARGET_MODEL_NAMES = ["target_fixed", "target_wheel", "target_moving"]
NAV_TIMEOUT = 120

# ============ 射击瞄准参数 ============
PID_KP = 5.5
ANGULAR_MAX = 0.5
HORIZONTAL_TOLERANCE = 15
AIM_TIMEOUT = 30
SHOOT_SERVICE = "/shoot_sim/shoot"


def euler_to_quaternion(yaw_deg):
    yaw = radians(yaw_deg)
    q = Quaternion()
    q.x, q.y = 0, 0
    q.z = sin(yaw / 2)
    q.w = cos(yaw / 2)
    return q


class Shoot1Only:
    def __init__(self):
        rospy.init_node("shoot_race_shoot1_only", anonymous=False)

        self.arrival_pub = rospy.Publisher("/shoot_race/arrival", String, queue_size=1)
        self.spawn_pub = rospy.Publisher("/shoot_race/spawn_task_board", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=False)
        self._goal_status = None

        self._model_states = None
        self._target_center = None
        self._camera_center_u = 400
        self._camera_center_v = 400

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)
        rospy.Subscriber("/target_center_pixel", PointStamped, self._on_target_center, queue_size=1)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._on_goal_status, queue_size=1)

    def _on_model_states(self, msg):
        self._model_states = msg

    def _on_target_center(self, msg):
        self._target_center = (msg.point.x, msg.point.y)

    def _on_camera_info(self, msg):
        if len(msg.K) >= 6:
            self._camera_center_u = msg.K[2]
            self._camera_center_v = msg.K[5]

    def _on_goal_status(self, msg):
        if msg.status_list:
            self._goal_status = msg.status_list[-1].status

    def wait_for_move_base(self, timeout=60):
        rospy.loginfo("[shoot1] 等待 move_base...")
        try:
            rospy.wait_for_message("/move_base/status", GoalStatusArray, timeout=timeout)
        except rospy.ROSException:
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

    def _pid_angular(self, error_u):
        out = PID_KP * error_u
        out = max(-ANGULAR_MAX, min(ANGULAR_MAX, out))
        return -out

    def _do_shoot(self):
        try:
            shoot_srv = rospy.ServiceProxy(SHOOT_SERVICE, Trigger)
            resp = shoot_srv()
            return resp.success if resp else False
        except Exception as e:
            rospy.logerr("[shoot1] 射击服务调用失败: %s", e)
            return False

    def shoot_at_ring_target(self):
        """环形靶：横向 PID 对准后射击"""
        rospy.loginfo("[shoot1] 瞄准环形靶...")
        self._target_center = None
        rate = rospy.Rate(20)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > AIM_TIMEOUT:
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
            ang = self._pid_angular(error_u)
            if abs(error_u) < HORIZONTAL_TOLERANCE:
                self._stop_robot()
                rospy.sleep(0.2)
                if self._do_shoot():
                    rospy.loginfo("[shoot1] 射击完成")
                    return True
                rospy.sleep(0.5)
                continue
            t = Twist()
            t.angular.z = ang
            self.cmd_vel_pub.publish(t)
            rate.sleep()
        return False

    def goto_goal(self, x, y, yaw_deg):
        """发布导航目标并等待到达"""
        self._goal_status = None
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0
        goal.pose.orientation = euler_to_quaternion(yaw_deg)
        rospy.loginfo("[shoot1] 发布导航目标 (%.2f, %.2f) yaw=%.0f°", x, y, yaw_deg)
        self.goal_pub.publish(goal)
        rospy.sleep(0.5)
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        goal_received = False
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > NAV_TIMEOUT:
                rospy.logwarn("[shoot1] 导航超时")
                return False
            s = self._goal_status
            if s in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                goal_received = True
            if goal_received and s == GoalStatus.SUCCEEDED:
                rospy.loginfo("[shoot1] 目标已到达")
                return True
            if (rospy.Time.now() - start).to_sec() > 1.0 and s == GoalStatus.SUCCEEDED:
                rospy.loginfo("[shoot1] 目标已到达")
                return True
            if s in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
                rospy.logwarn("[shoot1] move_base 返回 %s", s)
                return False
            rate.sleep()
        return False

    def run(self, spawn_msg):
        if not self.wait_for_move_base():
            return
        x, y, yaw = SHOOT_1_GOAL
        rospy.loginfo("[shoot1] 前往 shoot_1 (%.2f, %.2f)", x, y)
        if not self.goto_goal(x, y, yaw):
            rospy.logwarn("[shoot1] 未到达 shoot_1")
            return
        self.shoot_at_ring_target()
        self.send_arrival("shoot_1")
        rospy.loginfo("[shoot1] 任务完成，可查看 /referee/score")


def main():
    print("=" * 50)
    print("射击比赛仿真 - 简化版（仅 shoot_1）")
    print("=" * 50)
    print("请输入靶子配置（格式 target2,target3,wheel_target）：")
    print("  例如: c,g,3")
    print("-" * 50)
    spawn_msg = input(">>> ").strip() or "c,g,3"
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

    ref.run(spawn_msg)


if __name__ == "__main__":
    main()
