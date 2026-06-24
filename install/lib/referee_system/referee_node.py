#!/usr/bin/env python3
"""
裁判系统：
- 订阅 /shoot_targets/hit，解析命中消息，累计得分
- 订阅 /shoot_race/arrival，参赛方到达任务点/终点时发送，裁判计分
- 订阅 /shoot_race/spawn_task_board，参赛方比赛开始时发布：触发生成识别任务板；计时从 3 个靶子模型加载完成后开始
- 收到 finish 到达时计算总用时并发布到 ~elapsed_time
- 射击任务点：按机器人四轮形成的矩形与任务点框的重叠程度给分，完全进入 10 分，部分进入按比例（整数）
"""
import math
import os
import random
import re
import shutil
import sys

import rospy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerResponse
from gazebo_msgs.msg import ModelStates, ContactsState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel


# 到达点位话题：参赛方到达时发送 std_msgs/String，裁判按机器人 Gazebo 位姿验证
#   shoot_1 - 射击任务点1，在区域内 10 分
#   shoot_2 - 射击任务点2，在区域内 10 分
#   shoot_3 - 射击任务点3，在区域内 10 分
#   finish  - 终点，在 10 分区域 10 分、5 分区域 5 分
# 障碍区域 A/B 由裁判按机器人位姿自动判定，无需 arrival 消息
ARRIVAL_TOPIC = "/shoot_race/arrival"
SPAWN_TASK_BOARD_TOPIC = "/shoot_race/spawn_task_board"  # 比赛开始：格式 target1,target2,target3,wheel_target（平移靶目标由任务板图片决定）

# release 版（PyInstaller 打包）：参数嵌入二进制，忽略 launch 中的参数，防止参赛者修改
try:
    from release_params import RELEASE_PARAMS
except ImportError:
    RELEASE_PARAMS = {}


def _get_param(name: str, default):
    """frozen 时使用嵌入参数，否则从 launch 读取"""
    if getattr(sys, "frozen", False) and RELEASE_PARAMS and name in RELEASE_PARAMS:
        return RELEASE_PARAMS[name]
    return rospy.get_param("~" + name, default)


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _make_task_board_sdf(model_name: str, material_path: str, width: float, height: float) -> str:
    """生成识别任务板 SDF，板面贴图由 material 指定。width/height 为板面宽高(m)"""
    t = 0.01  # 厚度 1cm
    material_block = (
        "          <script><uri>file://"
        + material_path
        + "</uri><name>TaskBoardTexture</name></script>"
    )
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='link'>
      <kinematic>true</kinematic>
      <collision name='board_collision'>
        <geometry><box><size>{width} {t} {height}</size></box></geometry>
      </collision>
      <visual name='board_visual'>
        <geometry><box><size>{width} {t} {height}</size></box></geometry>
        <material>
{material_block}
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _polygon_area(points: list) -> float:
    """Shoelace 公式计算多边形面积"""
    if len(points) < 3:
        return 0.0
    n = len(points)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += points[i][0] * points[j][1]
        area -= points[j][0] * points[i][1]
    return abs(area) * 0.5


def _clip_polygon_by_rect(poly: list, cx: float, cy: float, hw: float, hh: float) -> list:
    """Sutherland-Hodgman：用轴对齐矩形裁剪多边形，返回裁剪后的顶点列表"""
    def inside(p, edge):
        # edge: 0=左(x>=cx-hw), 1=右(x<=cx+hw), 2=下(y>=cy-hh), 3=上(y<=cy+hh)
        x, y = p[0], p[1]
        if edge == 0:
            return x >= cx - hw
        if edge == 1:
            return x <= cx + hw
        if edge == 2:
            return y >= cy - hh
        return y <= cy + hh

    def intersect(p1, p2, edge):
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        if edge == 0:
            t = (cx - hw - x1) / (x2 - x1) if abs(x2 - x1) > 1e-9 else 0
        elif edge == 1:
            t = (cx + hw - x1) / (x2 - x1) if abs(x2 - x1) > 1e-9 else 0
        elif edge == 2:
            t = (cy - hh - y1) / (y2 - y1) if abs(y2 - y1) > 1e-9 else 0
        else:
            t = (cy + hh - y1) / (y2 - y1) if abs(y2 - y1) > 1e-9 else 0
        t = max(0, min(1, t))
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    out = poly[:]
    for edge in range(4):
        if not out:
            break
        inp, out = out, []
        for i in range(len(inp)):
            p1, p2 = inp[i], inp[(i + 1) % len(inp)]
            in1, in2 = inside(p1, edge), inside(p2, edge)
            if in1 and in2:
                out.append(p2)
            elif in1 and not in2:
                out.append(intersect(p1, p2, edge))
            elif not in1 and in2:
                out.append(intersect(p1, p2, edge))
                out.append(p2)
    return out


def _parse_hit(data: str) -> tuple:
    """
    解析命中消息，返回 (target_type, value) 或 (None, 0)
    target_type: 'fixed'|'wheel'|'moving'
    value: 固定靶为环数6~10，旋转靶为叶片1~5，平移靶为区域1~3
    """
    s = (data or "").strip()
    m = re.match(r"fixed\s+score=(\d+)\s+bullet=", s)
    if m:
        return "fixed", int(m.group(1))
    m = re.match(r"wheel\s+leaf=(\d+)\s+bullet=", s)
    if m:
        return "wheel", int(m.group(1))
    m = re.match(r"moving\s+region=(\d+)\s+bullet=", s)
    if m:
        return "moving", int(m.group(1))
    return None, 0


class RefereeNode:
    def __init__(self):
        rospy.init_node("referee", anonymous=False)

        # 计分规则：1号固定靶无系数、位置固定；2/3号靶位置 c=×1，g=×2
        # - 1号固定靶：环数(6~10)，上限10分，无系数
        # - 2号旋转靶：仅打中指定叶片(1~5)得5×系数(最高10分)，否则0
        # - 3号平移靶：仅打中指定区域(1~3)得5×系数(最高10分)，否则0；目标由任务板图片决定
        self._target_2_pos = "camouflage"
        self._target_3_pos = "camouflage"
        self._wheel_target_leaf = 1   # 1~5，打中此叶片才得10分
        self._moving_target_region = 1  # 1~3，由任务板图片决定：ak47/helmet/pack→1，aid/gauze/iv→2，mag/box/belt→3

        self._total_score = 0
        self._hit_count = 0
        self._arrived = set()  # 已计分的到达点位，防止重复
        self._target_hit = {"fixed": False, "wheel": False, "moving": False}  # 射击靶子只计第一次命中
        self._score_breakdown = []  # [(环节名, 分数, 原因), ...] 用于生成成绩单
        self._report_written = False  # 防止 finish 重复触发时多次写入
        self._lock = __import__("threading").Lock()

        # 分数参数：frozen 时从嵌入配置读取，否则从 launch 读取
        self._score_shoot_point_max = int(_get_param("score_shoot_point_max", 10))
        self._score_finish_5_max = int(_get_param("score_finish_5_max", 5))
        self._score_finish_10_max = int(_get_param("score_finish_10_max", 5))
        self._score_ob_a_half_max = int(_get_param("score_ob_a_half_max", 5))
        self._score_ob_b_max = int(_get_param("score_ob_b_max", 10))
        self._score_fixed_max = int(_get_param("score_fixed_max", 10))
        self._score_wheel_max = int(_get_param("score_wheel_max", 10))
        self._score_moving_max = int(_get_param("score_moving_max", 10))

        # 射击任务点区域：50×35cm，中心(x,y)+半宽半高
        self._shoot_regions = {
            "shoot_1": (
                float(_get_param("task_point_1_x", 0)),
                float(_get_param("task_point_1_y", 0)),
                float(_get_param("task_point_1_half_w", 0.25)),
                float(_get_param("task_point_1_half_h", 0.175)),
            ),
            "shoot_2": (
                float(_get_param("task_point_2_x", 0)),
                float(_get_param("task_point_2_y", 0)),
                float(_get_param("task_point_2_half_w", 0.25)),
                float(_get_param("task_point_2_half_h", 0.175)),
            ),
            "shoot_3": (
                float(_get_param("task_point_3_x", 0)),
                float(_get_param("task_point_3_y", 0)),
                float(_get_param("task_point_3_half_w", 0.25)),
                float(_get_param("task_point_3_half_h", 0.175)),
            ),
        }

        # 终点区域：10分区域（内圈）、5分区域（外圈）
        self._finish_10_cx = float(_get_param("finish_10_x", 0))
        self._finish_10_cy = float(_get_param("finish_10_y", 0))
        self._finish_10_hw = float(_get_param("finish_10_half_w", 0.25))
        self._finish_10_hh = float(_get_param("finish_10_half_h", 0.175))
        self._finish_5_cx = float(_get_param("finish_5_x", 0))
        self._finish_5_cy = float(_get_param("finish_5_y", 0))
        self._finish_5_hw = float(_get_param("finish_5_half_w", 0.45))
        self._finish_5_hh = float(_get_param("finish_5_half_h", 0.45))

        self._robot_name = _get_param("robot_model_name", "abot_model")
        self._robot_hw = float(_get_param("robot_half_w", 0.166))
        self._robot_hh = float(_get_param("robot_half_h", 0.155))
        self._robot_pose = None  # 缓存 /gazebo/model_states 中的机器人位姿

        # 障碍区域 A：避障，平分为前后两半，每半无碰撞得 5 分，共 10 分，只计一次
        self._ob_a_cx = float(_get_param("ob_a_x", 0))
        self._ob_a_cy = float(_get_param("ob_a_y", 0))
        self._ob_a_hw = float(_get_param("ob_a_half_w", 0.25))
        self._ob_a_hh = float(_get_param("ob_a_half_h", 0.175))
        self._ob_a_split = (_get_param("ob_a_split_axis", "x") or "x").lower()[:1]
        self._ob_a_first_scored = False
        self._ob_a_second_scored = False
        self._ob_a_first_entered = False   # 是否进入过前半区
        self._ob_a_second_entered = False  # 是否进入过后半区
        self._ob_a_in_first = False
        self._ob_a_in_second = False
        self._ob_a_prev_in_first = False
        self._ob_a_prev_in_second = False
        self._ob_a_first_collided = False
        self._ob_a_second_collided = False

        # 障碍区域 B：越障，经过整个区域得 10 分，只计一次
        self._ob_b_cx = float(_get_param("ob_b_x", 0))
        self._ob_b_cy = float(_get_param("ob_b_y", 0))
        self._ob_b_hw = float(_get_param("ob_b_half_w", 0.25))
        self._ob_b_hh = float(_get_param("ob_b_half_h", 0.175))
        self._ob_b_scored = False
        self._ob_b_entered = False

        # 区域 A 碰撞检测：map_bumper + abot_bumper 双源
        self._ob_a_map_bumper_topic = _get_param("ob_a_map_bumper_topic", "/map_bumper")
        self._ob_a_abot_bumper_topic = _get_param("ob_a_abot_bumper_topic", "/abot_bumper")
        self._ob_a_has_non_ground_contact = False

        self._score_pub = rospy.Publisher("~score", Int32, queue_size=10, latch=True)
        self._score_pub.publish(Int32(data=0))
        self._start_time = None  # 3 个靶子模型加载完成后记录，finish 时打印用时
        self._waiting_for_target_models = False  # spawn 收到后为 True，模型加载完才设 _start_time
        self._target_model_names = ["target_fixed", "target_wheel", "target_moving"]

        # 识别任务板：收到 spawn 话题后随机选图生成
        self._task_board_name = _get_param("task_board_name", "recognition_task_board")
        self._task_board_x = float(_get_param("task_board_x", 2.0))
        self._task_board_y = float(_get_param("task_board_y", 1.0))
        self._task_board_z = float(_get_param("task_board_z", 0.5))
        self._task_board_yaw = float(_get_param("task_board_yaw", 0.0))
        self._task_board_width = float(_get_param("task_board_width", 0.21))
        self._task_board_height = float(_get_param("task_board_height", 0.297))
        self._pkg_name = _get_param("package_name", "referee_system")
        # PyInstaller 打包时图片嵌入二进制，从 sys._MEIPASS 读取，防止被替换
        if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
            self._images_dir = os.path.join(sys._MEIPASS, "images")
        else:
            try:
                import rospkg
                pkg_path = rospkg.RosPack().get_path(self._pkg_name)
                self._images_dir = os.path.join(pkg_path, "images")
            except Exception:
                self._images_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        if not os.path.isdir(self._images_dir):
            rospy.logwarn("[裁判] 识别任务板图片目录不存在: %s", self._images_dir)
        # 材质写入包内 materials，使用 rospkg 路径，部署时无硬编码；包只读时回退到 ~/.referee_task_board
        try:
            pkg_mat = os.path.join(rospkg.RosPack().get_path(self._pkg_name), "materials")
            fallback = os.path.join(os.path.expanduser("~"), ".referee_task_board")
            for base in (pkg_mat, fallback):
                scripts_d = os.path.join(base, "scripts")
                textures_d = os.path.join(base, "textures")
                try:
                    os.makedirs(scripts_d, exist_ok=True)
                    os.makedirs(textures_d, exist_ok=True)
                    # 测试写入权限
                    test_file = os.path.join(textures_d, ".write_test")
                    with open(test_file, "w"):
                        pass
                    os.remove(test_file)
                    self._materials_base = base
                    break
                except (OSError, IOError):
                    continue
            else:
                self._materials_base = fallback  # 最后尝试 fallback
        except Exception:
            self._materials_base = os.path.join(os.path.expanduser("~"), ".referee_task_board")
        self._task_board_images = [
            f for f in os.listdir(self._images_dir)
            if f.lower().endswith((".jpg", ".jpeg", ".png"))
        ] if os.path.isdir(self._images_dir) else []

        rospy.Subscriber("/shoot_targets/hit", String, self._on_hit, queue_size=50)
        rospy.Subscriber(ARRIVAL_TOPIC, String, self._on_arrival, queue_size=10)
        rospy.Subscriber(SPAWN_TASK_BOARD_TOPIC, String, self._on_spawn_task_board, queue_size=5)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=5)
        rospy.Subscriber(self._ob_a_map_bumper_topic, ContactsState, self._on_bumper_contact, queue_size=10)
        rospy.Subscriber(self._ob_a_abot_bumper_topic, ContactsState, self._on_bumper_contact, queue_size=10)
        rospy.Service("~reset", Trigger, self._on_reset)
        rospy.Timer(rospy.Duration(0.05), self._check_obstacle_regions)  # 20Hz 检查障碍区域

        rospy.loginfo(
            "[裁判] 已启动，订阅 %s、%s、/shoot_targets/hit、/gazebo/model_states、%s、%s，发布 ~score",
            ARRIVAL_TOPIC, SPAWN_TASK_BOARD_TOPIC, self._ob_a_map_bumper_topic, self._ob_a_abot_bumper_topic
        )

    def _record_score(self, item: str, score: int, reason: str):
        """记录分数明细，用于生成成绩单"""
        self._score_breakdown.append((item, score, reason))

    def _write_score_report(self, elapsed: float):
        """收到 finish 时生成成绩单 txt 文件，保存到裁判系统包目录"""
        candidates = []
        try:
            import rospkg
            candidates.append(rospkg.RosPack().get_path(self._pkg_name))
        except Exception:
            pass
        candidates.append(os.path.expanduser("~"))
        sep = "=" * 50
        line = "-" * 50
        lines = [
            sep,
            "射击比赛仿真 - 比赛成绩单",
            sep,
            "",
            "总分: %d" % self._total_score,
            "用时: %.2f 秒" % elapsed,
            "",
            line,
            "各环节得分明细",
            line,
            "",
        ]
        for item, score, reason in self._score_breakdown:
            lines.append("【%s】 %d 分" % (item, score))
            lines.append("  原因: %s" % reason)
            lines.append("")
        lines.append(sep)
        content = "\n".join(lines)
        out_path = None
        for out_dir in candidates:
            p = os.path.join(out_dir, "match_score_report.txt")
            try:
                with open(p, "w", encoding="utf-8") as f:
                    f.write(content)
                out_path = p
                break
            except (OSError, IOError):
                continue
        if out_path:
            rospy.loginfo("[裁判] 成绩单已保存: %s", out_path)
        else:
            rospy.logwarn("[裁判] 保存成绩单失败，无法写入任何目录")

    def _on_model_states(self, msg: ModelStates):
        try:
            i = msg.name.index(self._robot_name)
            self._robot_pose = msg.pose[i]
        except (ValueError, IndexError):
            pass
        # 计时从 3 个靶子模型加载完成后开始
        if self._waiting_for_target_models and self._start_time is None:
            names = set(msg.name)
            if all(n in names for n in self._target_model_names):
                with self._lock:
                    self._start_time = rospy.Time.now()
                    self._waiting_for_target_models = False
                rospy.loginfo("[裁判] 靶子模型已加载，开始计时")

    _OB_A_CONTACT_Z_THRESH = 0.008  # 接触点 z>此值视为墙体（轮子压地 z≈±1e-5）

    def _contact_in_region_a(self, px: float, py: float) -> bool:
        """接触点 (px,py) 是否在区域 A 内"""
        return (abs(px - self._ob_a_cx) <= self._ob_a_hw and
                abs(py - self._ob_a_cy) <= self._ob_a_hh)

    def _contact_in_first_half(self, px: float, py: float) -> bool:
        """接触点是否在前半区，与 _half_rect first=True 一致。split=x: 前半左半 x<cx；split=y: 前半下半 y<cy"""
        sp = self._ob_a_split
        if sp == "x":
            return px < self._ob_a_cx
        return py < self._ob_a_cy

    def _on_bumper_contact(self, msg: ContactsState):
        """map_bumper/abot_bumper：机器人-地图接触，接触点在区域 A 内，且(法向水平|n.z|<0.7 或 接触z>阈值)则记为撞墙。按接触点所在半区设置 first/second_collided。"""
        for state in msg.states:
            c1 = (state.collision1_name or "")
            c2 = (state.collision2_name or "")
            if self._robot_name not in c1 and self._robot_name not in c2:
                continue
            if "shoot_race_map" not in c1 and "shoot_race_map" not in c2:
                continue
            positions = state.contact_positions or []
            norms = state.contact_normals or []
            for i, p in enumerate(positions):
                if not self._contact_in_region_a(p.x, p.y):
                    continue
                nz = abs(norms[i].z) if i < len(norms) else 1.0
                is_wall = nz < 0.7 or p.z > self._OB_A_CONTACT_Z_THRESH
                if not is_wall:
                    rospy.loginfo_throttle(2.0, "[裁判] 区域A内 abot-map 接触未记撞墙: %s n.z=%.3f p.z=%.5f",
                                          c1[:50], norms[i].z if i < len(norms) else 0, p.z)
                    continue
                with self._lock:
                    self._ob_a_has_non_ground_contact = True
                    if self._contact_in_first_half(p.x, p.y):
                        self._ob_a_first_collided = True
                        half_name = "前半区"
                    else:
                        self._ob_a_second_collided = True
                        half_name = "后半区"
                rospy.loginfo("[裁判] 检测到区域A%s墙体碰撞 (接触点 %.2f,%.2f) n.z=%.3f",
                             half_name, p.x, p.y, norms[i].z if i < len(norms) else 0)
                return

    def _robot_has_non_ground_collision(self) -> bool:
        """机器人底盘是否发生除底面外的碰撞（墙面等）"""
        return self._ob_a_has_non_ground_contact

    def _robot_in_rect(self, cx: float, cy: float, hw: float, hh: float) -> bool:
        """机器人中心是否在矩形内"""
        if self._robot_pose is None:
            return False
        x, y = self._robot_pose.position.x, self._robot_pose.position.y
        return abs(x - cx) <= hw and abs(y - cy) <= hh

    def _robot_rect_overlaps_rect(self, cx: float, cy: float, hw: float, hh: float) -> bool:
        """机器人四轮矩形是否与区域有重叠（进入过该区域）"""
        corners = self._robot_rect_corners()
        if not corners:
            return False
        clipped = _clip_polygon_by_rect(corners, cx, cy, hw, hh)
        return len(clipped) >= 3 and _polygon_area(clipped) > 1e-6

    def _robot_rect_fully_outside_rect(self, cx: float, cy: float, hw: float, hh: float) -> bool:
        """机器人四轮矩形投影是否完全在区域外（与区域无重叠）"""
        return not self._robot_rect_overlaps_rect(cx, cy, hw, hh)

    def _robot_in_rect_half(self, cx: float, cy: float, hw: float, hh: float, split: str, first: bool) -> bool:
        """机器人中心是否在前半/后半，与 _half_rect 一致。split=x: 前半左半 x<cx；split=y: 前半下半 y<cy"""
        if self._robot_pose is None:
            return False
        x, y = self._robot_pose.position.x, self._robot_pose.position.y
        if not (abs(x - cx) <= hw and abs(y - cy) <= hh):
            return False
        if split == "x":
            in_first = x < cx
        else:
            in_first = y < cy
        return in_first if first else (not in_first)

    def _half_rect(self, cx: float, cy: float, hw: float, hh: float, split: str, first: bool) -> tuple:
        """返回半区矩形 (cx, cy, hw, hh)。split=x: 前半左半，后半右半；split=y: 前半下半，后半上半（前半=先经过的）"""
        if split == "x":
            hw2 = hw / 2
            if first:
                return (cx - hw2, cy, hw2, hh)
            return (cx + hw2, cy, hw2, hh)
        hh2 = hh / 2
        if first:
            return (cx, cy - hh2, hw, hh2)
        return (cx, cy + hh2, hw, hh2)

    def _check_obstacle_regions(self, _evt):
        """定时检查障碍区域 A/B，自动计分"""
        if self._robot_pose is None:
            return
        with self._lock:
            # 区域 B：机器人进入后，投影完全离开才得 10 分，只计一次
            if not self._ob_b_entered and self._robot_rect_overlaps_rect(
                self._ob_b_cx, self._ob_b_cy, self._ob_b_hw, self._ob_b_hh
            ):
                self._ob_b_entered = True
            if self._ob_b_entered and not self._ob_b_scored and self._robot_rect_fully_outside_rect(
                self._ob_b_cx, self._ob_b_cy, self._ob_b_hw, self._ob_b_hh
            ):
                self._ob_b_scored = True
                add = self._score_ob_b_max
                self._total_score += add
                self._score_pub.publish(Int32(data=self._total_score))
                self._record_score("障碍区域B（越障）", add, "机器人进入后完全离开区域，无碰撞")
                rospy.loginfo("[裁判] 通过障碍区域 B（完全离开） +%d 总分=%d", add, self._total_score)

            # 区域 A：分前后两半，每半需完全通过（进入后投影完全离开）且无碰撞才得 5 分
            if self._ob_a_first_scored and self._ob_a_second_scored:
                self._ob_a_prev_in_first = False
                self._ob_a_prev_in_second = False
                return
            hcx, hcy, hhw, hhh = self._ob_a_cx, self._ob_a_cy, self._ob_a_hw, self._ob_a_hh
            sp = self._ob_a_split
            # 前半区、后半区矩形 (cx, cy, hw, hh)
            first_rect = self._half_rect(hcx, hcy, hhw, hhh, sp, first=True)
            second_rect = self._half_rect(hcx, hcy, hhw, hhh, sp, first=False)
            in_first = self._robot_in_rect_half(hcx, hcy, hhw, hhh, sp, first=True)
            in_second = self._robot_in_rect_half(hcx, hcy, hhw, hhh, sp, first=False)
            self._ob_a_in_first = in_first
            self._ob_a_in_second = in_second
            # 碰撞由 bumper 回调按接触点所在半区设置 first/second_collided
            # 前半：进入过（重叠过）且投影完全离开前半区才结算
            if not self._ob_a_first_entered and self._robot_rect_overlaps_rect(*first_rect):
                self._ob_a_first_entered = True
            if self._ob_a_first_entered and not self._ob_a_first_scored and self._robot_rect_fully_outside_rect(*first_rect):
                no_collision = not self._ob_a_first_collided
                self._ob_a_first_scored = True
                if no_collision:
                    add = self._score_ob_a_half_max
                    self._total_score += add
                    self._score_pub.publish(Int32(data=self._total_score))
                    self._record_score("障碍区域A前半区", add, "经过前半区无碰撞")
                    rospy.loginfo("[裁判] 区域A前半区(无碰撞) +%d 总分=%d", add, self._total_score)
                else:
                    self._record_score("障碍区域A前半区", 0, "经过前半区有碰撞，0分")
                    rospy.loginfo("[裁判] 区域A前半区(有碰撞) 0分 总分=%d", self._total_score)
                self._ob_a_first_collided = False
                self._ob_a_has_non_ground_contact = False
            # 后半：进入过且投影完全离开后半区才给分
            if not self._ob_a_second_entered and self._robot_rect_overlaps_rect(*second_rect):
                self._ob_a_second_entered = True
            if self._ob_a_second_entered and not self._ob_a_second_scored and self._robot_rect_fully_outside_rect(*second_rect):
                no_collision = not self._ob_a_second_collided
                self._ob_a_second_scored = True
                if no_collision:
                    add = self._score_ob_a_half_max
                    self._total_score += add
                    self._score_pub.publish(Int32(data=self._total_score))
                    self._record_score("障碍区域A后半区", add, "经过后半区无碰撞")
                    rospy.loginfo("[裁判] 区域A后半区(无碰撞) +%d 总分=%d", add, self._total_score)
                else:
                    self._record_score("障碍区域A后半区", 0, "经过后半区有碰撞，0分")
                    rospy.loginfo("[裁判] 区域A后半区(有碰撞) 0分 总分=%d", self._total_score)
                self._ob_a_second_collided = False
                self._ob_a_has_non_ground_contact = False
            self._ob_a_prev_in_first = in_first
            self._ob_a_prev_in_second = in_second

    def _parse_spawn_message(self, data: str) -> None:
        """解析 spawn_task_board 消息：target2,target3,wheel_target（1号固定靶位置固定、无系数）"""
        s = (data or "").strip()
        parts = [p.strip().lower() for p in s.split(",")]
        if len(parts) >= 2:
            def _pos(p):
                if p in ("g", "gray", "grey", "灰色"):
                    return "gray"
                return "camouflage"
            self._target_2_pos = _pos(parts[0])
            self._target_3_pos = _pos(parts[1])
        if len(parts) >= 3:
            try:
                w = int(parts[2])
                self._wheel_target_leaf = max(1, min(5, w))
            except (ValueError, TypeError):
                self._wheel_target_leaf = 1
        rospy.loginfo("[裁判] 靶子位置 2=%s 3=%s，旋转靶目标叶片=%d",
                      self._target_2_pos, self._target_3_pos, self._wheel_target_leaf)

    def _moving_target_from_image(self, img_name: str) -> int:
        """根据任务板图片确定平移靶目标区域：ak47/helmet/pack→1，aid/gauze/iv→2，mag/box/belt→3"""
        base = os.path.splitext(img_name)[0].lower()
        if base in ("ak47", "helmet", "pack"):
            return 1
        if base in ("aid", "gauze", "iv"):
            return 2
        if base in ("mag", "box", "belt"):
            return 3
        return 1  # 未知图片默认 1

    def _coef(self, pos: str) -> int:
        """c=1, g=2"""
        return 2 if pos == "gray" else 1

    def _on_spawn_task_board(self, msg: String):
        """收到 spawn 话题后，解析靶子参数，随机选图在仿真中生成识别任务板。计时从 3 个靶子模型加载完成后开始。"""
        self._waiting_for_target_models = True
        self._parse_spawn_message(msg.data if msg else "")
        with self._lock:
            self._score_breakdown = []
            self._report_written = False
            self._target_hit = {"fixed": False, "wheel": False, "moving": False}
            self._ob_a_first_scored = False
            self._ob_a_second_scored = False
            self._ob_a_first_entered = False
            self._ob_a_second_entered = False
            self._ob_a_prev_in_first = False
            self._ob_a_prev_in_second = False
            self._ob_a_first_collided = False
            self._ob_a_second_collided = False
            self._ob_b_scored = False
            self._ob_b_entered = False
        if not self._task_board_images:
            rospy.logwarn("[裁判] 无可用图片，无法生成识别任务板")
            return
        img_name = random.choice(self._task_board_images)
        self._moving_target_region = self._moving_target_from_image(img_name)
        rospy.loginfo("[裁判] 任务板图片 %s → 平移靶目标区域=%d", img_name, self._moving_target_region)
        img_path = os.path.abspath(os.path.join(self._images_dir, img_name))
        if not os.path.isfile(img_path):
            rospy.logwarn("[裁判] 图片不存在: %s", img_path)
            return
        scripts_dir = os.path.join(self._materials_base, "scripts")
        textures_dir = os.path.join(self._materials_base, "textures")
        for d in (scripts_dir, textures_dir):
            try:
                os.makedirs(d, exist_ok=True)
            except OSError as e:
                rospy.logerr("[裁判] 创建目录失败 %s: %s", d, e)
                return
        ext = os.path.splitext(img_name)[1].lower()
        if ext not in (".jpg", ".jpeg", ".png"):
            ext = ".jpg"
        tex_name = "task_board" + ext
        tex_path = os.path.join(textures_dir, tex_name)
        try:
            shutil.copy2(img_path, tex_path)
        except Exception as e:
            rospy.logerr("[裁判] 复制图片失败: %s", e)
            return
        mat_path = os.path.join(scripts_dir, "task_board.material")
        try:
            with open(mat_path, "w") as f:
                f.write("material TaskBoardTexture\n{\n")
                f.write("  technique { pass { texture_unit { texture ../textures/")
                f.write(tex_name)
                f.write(" } } }\n}\n")
        except Exception as e:
            rospy.logerr("[裁判] 写入 material 失败: %s", e)
            return
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=5.0)
            rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
            spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            try:
                delete_srv(self._task_board_name)
                rospy.sleep(0.3)
            except Exception:
                pass
            pose = Pose()
            pose.position.x = self._task_board_x
            pose.position.y = self._task_board_y
            pose.position.z = self._task_board_z
            pose.orientation = _quat_from_rpy(0.0, 0.0, self._task_board_yaw)
            sdf = _make_task_board_sdf(
                self._task_board_name,
                mat_path,
                self._task_board_width,
                self._task_board_height,
            )
            req = SpawnModelRequest()
            req.model_name = self._task_board_name
            req.model_xml = sdf
            req.reference_frame = ""
            req.initial_pose = pose
            resp = spawn_srv(req)
            if resp.success:
                rospy.loginfo("[裁判] 识别任务板已生成，图片: %s 位置: (%.2f, %.2f, %.2f)",
                              img_name, self._task_board_x, self._task_board_y, self._task_board_z)
            else:
                rospy.logwarn("[裁判] 生成任务板失败: %s", getattr(resp, "status_message", ""))
        except Exception as e:
            rospy.logerr("[裁判] 生成任务板异常: %s", e)

    def _robot_rect_corners(self) -> list:
        """机器人四轮矩形的四个角点（世界坐标系）"""
        if self._robot_pose is None:
            return []
        q = self._robot_pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-9:
            return []
        x, y, z, w = x / n, y / n, z / n, w / n
        # 旋转矩阵 (从机器人局部到世界，yaw)
        # 局部角点 (±hw, ±hh)
        cx = self._robot_pose.position.x
        cy = self._robot_pose.position.y
        hw, hh = self._robot_hw, self._robot_hh
        corners_local = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        # yaw from quaternion: atan2(2(wz+xy), 1-2(xx+yy))
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        cos_a, sin_a = math.cos(yaw), math.sin(yaw)
        out = []
        for lx, ly in corners_local:
            wx = cx + cos_a * lx - sin_a * ly
            wy = cy + sin_a * lx + cos_a * ly
            out.append((wx, wy))
        return out

    def _shoot_score_by_position(self, key: str) -> tuple:
        """按机器人四轮矩形与任务点框的重叠程度给分，返回 (分数, 原因)"""
        if self._robot_pose is None:
            rospy.logwarn("[裁判] %s 时未获取到机器人位姿，给 0 分", key)
            return 0, "未获取到机器人位姿"
        corners = self._robot_rect_corners()
        if not corners:
            return 0, "机器人矩形无效"
        cx, cy, hw, hh = self._shoot_regions[key]
        clipped = _clip_polygon_by_rect(corners, cx, cy, hw, hh)
        if len(clipped) < 3:
            rospy.loginfo("[裁判] %s 时机器人未进入任务点框，给 0 分", key)
            return 0, "机器人未进入任务点框"
        inter_area = _polygon_area(clipped)
        robot_area = 4.0 * self._robot_hw * self._robot_hh
        if robot_area < 1e-9:
            return 0, "机器人面积无效"
        ratio = inter_area / robot_area
        max_s = self._score_shoot_point_max
        score = min(max_s, max(0, round(1.0 * max_s * ratio)))
        rospy.loginfo("[裁判] %s 重叠比例=%.2f 得分=%d", key, ratio, score)
        name_map = {"shoot_1": "射击任务点1", "shoot_2": "射击任务点2", "shoot_3": "射击任务点3"}
        reason = "机器人与任务点框重叠比例 %.0f%%" % (ratio * 100)
        return int(score), reason

    def _finish_score_by_position(self) -> tuple:
        """终点：大框 0~5 分，小框 0~5 分，返回 (分数, 原因)"""
        if self._robot_pose is None:
            rospy.logwarn("[裁判] finish 时未获取到机器人位姿，给 0 分")
            return 0, "未获取到机器人位姿"
        corners = self._robot_rect_corners()
        if not corners:
            return 0, "机器人矩形无效"
        robot_area = 4.0 * self._robot_hw * self._robot_hh
        if robot_area < 1e-9:
            return 0, "机器人面积无效"
        max_5 = self._score_finish_5_max
        clipped_5 = _clip_polygon_by_rect(
            corners, self._finish_5_cx, self._finish_5_cy,
            self._finish_5_hw, self._finish_5_hh
        )
        inter_5 = _polygon_area(clipped_5) if len(clipped_5) >= 3 else 0.0
        ratio_5 = inter_5 / robot_area
        score_5 = min(max_5, max(0, round(1.0 * max_5 * ratio_5)))
        max_10 = self._score_finish_10_max
        score_10 = 0
        ratio_10 = 0.0
        if ratio_5 >= 0.999:
            clipped_10 = _clip_polygon_by_rect(
                corners, self._finish_10_cx, self._finish_10_cy,
                self._finish_10_hw, self._finish_10_hh
            )
            inter_10 = _polygon_area(clipped_10) if len(clipped_10) >= 3 else 0.0
            ratio_10 = inter_10 / robot_area
            score_10 = min(max_10, max(0, round(1.0 * max_10 * ratio_10)))
        total = min(max_5 + max_10, score_5 + score_10)
        rospy.loginfo("[裁判] finish 大框=%.0f%%(%d) 小框=%.0f%%(+%d) 总分=%d",
                      ratio_5 * 100, score_5, ratio_10 * 100, score_10, total)
        reason = "终点外框进入 %.0f%%(%d分)，内框进入 %.0f%%(+%d分)" % (ratio_5 * 100, score_5, ratio_10 * 100, score_10)
        return int(total), reason

    def _on_hit(self, msg: String):
        tt, val = _parse_hit(msg.data)
        if tt is None:
            return
        with self._lock:
            if self._target_hit.get(tt, False):
                add = 0
                rospy.loginfo("[裁判] 命中 %s=%s 重复，仅计第一次得分，忽略", tt, val)
            else:
                if tt == "fixed":
                    add = min(self._score_fixed_max, max(0, val))
                    reason = "固定靶环数 %d" % val
                elif tt == "wheel" and 1 <= val <= 5:
                    coef = self._coef(self._target_2_pos)
                    add = min(self._score_wheel_max, 5 * coef) if val == self._wheel_target_leaf else 0
                    reason = "旋转靶打中指定叶片%d" % val if val == self._wheel_target_leaf else "旋转靶打中非目标叶片%d，0分" % val
                elif tt == "moving" and 1 <= val <= 3:
                    coef = self._coef(self._target_3_pos)
                    add = min(self._score_moving_max, 5 * coef) if val == self._moving_target_region else 0
                    reason = "平移靶打中指定区域%d" % val if val == self._moving_target_region else "平移靶打中非目标区域%d，0分" % val
                else:
                    add, reason = 0, "无效命中"
                self._total_score += add
                self._hit_count += 1
                self._target_hit[tt] = True
                if add > 0:
                    name_map = {"fixed": "1号固定靶", "wheel": "2号旋转靶", "moving": "3号平移靶"}
                    self._record_score(name_map.get(tt, tt), add, reason)
        self._score_pub.publish(Int32(data=self._total_score))
        if add > 0:
            rospy.loginfo("[裁判] 命中 %s=%s +%d 总分=%d", tt, val, add, self._total_score)

    def _on_arrival(self, msg: String):
        key = (msg.data or "").strip().lower()
        valid = set(self._shoot_regions) | {"finish"}
        if key not in valid:
            rospy.logwarn("[裁判] 无效到达点位: %r，有效值: %s", msg.data, sorted(valid))
            return
        with self._lock:
            if key in self._arrived:
                if key != "finish":
                    rospy.loginfo("[裁判] 重复到达 %s，已计分过，忽略", key)
                    return
                add, reason = 0, ""
            else:
                if key == "finish":
                    add, reason = self._finish_score_by_position()
                else:
                    add, reason = self._shoot_score_by_position(key)
                if add > 0:
                    self._arrived.add(key)
                    self._total_score += add
                    name_map = {"shoot_1": "射击任务点1", "shoot_2": "射击任务点2", "shoot_3": "射击任务点3", "finish": "终点"}
                    self._record_score(name_map.get(key, key), add, reason)
                elif key == "finish":
                    self._arrived.add(key)
                    name_map = {"finish": "终点"}
                    self._record_score("终点", add, reason)
        if add > 0:
            self._score_pub.publish(Int32(data=self._total_score))
            rospy.loginfo("[裁判] 到达 %s +%d 总分=%d", key, add, self._total_score)
        if key == "finish":
            elapsed = (rospy.Time.now() - self._start_time).to_sec() if self._start_time else -1.0
            rospy.loginfo("[裁判] ========== 比赛结束 ========== 总分: %d  用时: %.2f 秒", self._total_score, elapsed)
            if not self._report_written:
                self._report_written = True
                self._write_score_report(elapsed)

    def _on_reset(self, _req) -> TriggerResponse:
        with self._lock:
            self._total_score = 0
            self._hit_count = 0
            self._arrived.clear()
            self._score_breakdown = []
            self._report_written = False
            self._target_hit = {"fixed": False, "wheel": False, "moving": False}
            self._ob_a_first_scored = False
            self._ob_a_second_scored = False
            self._ob_a_first_entered = False
            self._ob_a_second_entered = False
            self._ob_a_prev_in_first = False
            self._ob_a_prev_in_second = False
            self._ob_a_first_collided = False
            self._ob_a_second_collided = False
            self._ob_a_has_non_ground_contact = False
            self._ob_b_scored = False
            self._ob_b_entered = False
            self._start_time = None
            self._waiting_for_target_models = False
        self._score_pub.publish(Int32(data=0))
        rospy.loginfo("[裁判] 已重置")
        return TriggerResponse(success=True, message="Score reset to 0")

    def run(self):
        rospy.spin()


def main():
    n = RefereeNode()
    n.run()


if __name__ == "__main__":
    main()
