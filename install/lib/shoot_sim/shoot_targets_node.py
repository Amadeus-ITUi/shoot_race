#!/usr/bin/env python3
import math
import os
import sys
import threading
import time
from typing import Optional

import rospy

# race 版（PyInstaller 打包）：参数嵌入二进制，忽略 launch 中的参数
try:
    from release_params import RELEASE_PARAMS
except ImportError:
    RELEASE_PARAMS = {}


def _get_param(name: str, default):
    """frozen 时使用嵌入参数，否则从 launch 读取"""
    if getattr(sys, "frozen", False) and RELEASE_PARAMS and name in RELEASE_PARAMS:
        return RELEASE_PARAMS[name]
    return rospy.get_param("~" + name, default)


def _get_global_param(name: str, default):
    """如 /shoot_height 等全局参数"""
    if getattr(sys, "frozen", False) and RELEASE_PARAMS and name.lstrip("/") in RELEASE_PARAMS:
        return RELEASE_PARAMS[name.lstrip("/")]
    return rospy.get_param(name, default)


def _resolve_texture_path(path: str) -> str:
    """race 版：相对路径 materials/scripts/xxx 转为包内绝对路径"""
    if not path or not getattr(sys, "frozen", False) or "package_name" not in RELEASE_PARAMS:
        return path
    if os.path.isabs(path) or path.startswith("$("):
        return path
    try:
        import rospkg
        pkg_path = rospkg.RosPack().get_path(RELEASE_PARAMS["package_name"])
        return os.path.join(pkg_path, path)
    except Exception:
        return path
import yaml
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String


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


def _pose(x, y, z, yaw=0.0) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation = _quat_from_rpy(0.0, 0.0, float(yaw))
    return p


def _get_image_aspect_ratio(material_path: str) -> float:
    """从材质文件解析贴图路径，读取图片尺寸，返回 宽/高。失败则返回 1.0"""
    import os
    import re
    try:
        path = material_path
        if "$(find " in path:
            import rospkg
            pkg = re.search(r"\$\(find\s+(\w+)\)", path).group(1)
            base = rospkg.RosPack().get_path(pkg)
            path = path.replace("$(find %s)" % pkg, base)
        if not os.path.isfile(path):
            return 1.0
        with open(path, "r") as f:
            content = f.read()
        m = re.search(r'texture\s+([^\s\n]+)', content)
        if not m:
            return 1.0
        tex_rel = m.group(1).strip()
        mat_dir = os.path.dirname(path)
        tex_path = os.path.normpath(os.path.join(mat_dir, tex_rel))
        if not os.path.isfile(tex_path):
            return 1.0
        with open(tex_path, "rb") as f:
            header = f.read(24)
        if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
            return 1.0
        w = int.from_bytes(header[16:20], "big")
        h = int.from_bytes(header[20:24], "big")
        if h <= 0:
            return 1.0
        return float(w) / float(h)
    except Exception:
        return 1.0


def _pose_param(name: str, default_list):
    key = name.lstrip("~")
    if getattr(sys, "frozen", False) and RELEASE_PARAMS and key in RELEASE_PARAMS:
        v = RELEASE_PARAMS[key]
    else:
        v = rospy.get_param(name, default_list)
    if isinstance(v, str):
        try:
            v = yaml.safe_load(v)
        except Exception:
            v = default_list
    if not isinstance(v, (list, tuple)) or len(v) < 3:
        v = default_list
    if len(v) == 3:
        v = [v[0], v[1], v[2], 0.0]
    return [float(v[0]), float(v[1]), float(v[2]), float(v[3])]


def _model_exists(get_state_srv, name: str) -> bool:
    try:
        resp = get_state_srv(name, "world")
        return resp.success
    except Exception:
        return False


def _spawn_sdf(spawn_srv, delete_srv, get_state_srv, name: str, sdf: str, pose: Pose, retries: int = 10, delay_s: float = 0.5):
    # 仅当模型已存在时删除，避免 DeleteModel 对不存在的模型报 ERROR
    if _model_exists(get_state_srv, name):
        try:
            delete_srv(name)
            rospy.sleep(0.5)
        except Exception:
            pass
    req = SpawnModelRequest()
    req.model_name = name
    req.model_xml = sdf
    req.robot_namespace = rospy.get_namespace()
    req.reference_frame = ""  # Gazebo world
    req.initial_pose = pose
    last_err = None
    for _ in range(int(max(retries, 1))):
        try:
            resp = spawn_srv(req)
            if resp.success:
                return
            last_err = resp.status_message
            rospy.logwarn(f"spawn[{name}] failed: {last_err}")
        except Exception as e:
            last_err = str(e)
            rospy.logwarn(f"spawn[{name}] exception: {last_err}")
        rospy.sleep(float(delay_s))
    raise RuntimeError(last_err or "spawn failed")


def make_fixed_ring_target_sdf(model_name: str, scale: float = 1.0, texture: str = "") -> str:
    """固定靶：正方形面贴图，标准环靶 6~10 环(外环直径27cm)。texture 为 .material 路径，材质名 FixedTargetTexture"""
    s = float(scale)
    backing_t = 0.02 * s
    # fixed_scale=1 时正方形边长 0.27m，其它按 scale 等比缩放
    side = 0.27 * s
    backing_w = side
    backing_h = side
    if texture:
        material_block = (
            "          <script><uri>file://"
            + texture
            + "</uri><name>FixedTargetTexture</name></script>"
        )
    else:
        material_block = (
            "          <ambient>0.9 0.9 0.9 1</ambient>\n"
            "          <diffuse>0.9 0.9 0.9 1</diffuse>"
        )
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='link'>
      <collision name='backing_collision'>
        <geometry><box><size>{backing_t} {backing_w} {backing_h}</size></box></geometry>
      </collision>
      <visual name='target_visual'>
        <geometry><box><size>{backing_t} {backing_w} {backing_h}</size></box></geometry>
        <material>
{material_block}
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def make_ferris_wheel_target_sdf(
    model_name: str, blade_count: int = 5, scale: float = 1.0, textures: Optional[list] = None
) -> str:
    # Wheel base (visual only): hub + 5 简单的正方形面板，始终朝前，方便贴 AR 纹理。
    s = float(scale)
    # 以 wheel_scale=1 时，小正方形边长约 0.05m，距旋转中心约 0.09m，随 scale 等比缩放
    leaf_half_size = 0.025 * s  # 边长 = 0.05 * scale
    leaf_radius = 0.09 * s      # 半径 = 0.09 * scale
    blades_xml = []
    for i in range(blade_count):
        ang = 2.0 * math.pi * i / blade_count
        # Blade positions in wheel plane (Y-Z), wheel rotates around X.
        r = leaf_radius
        z = r * math.cos(ang)
        yy = r * math.sin(ang)
        color = ["1 0 0 1", "0 1 0 1", "0 0 1 1", "1 1 0 1", "1 0 1 1"][i % 5]
        tex = ""
        if textures and i < len(textures):
            tex = textures[i]
        if tex:
            mat_name = f"WheelLeaf{i + 1}Texture"
            material_block = (
                f"          <script><uri>file://{tex}</uri><name>{mat_name}</name></script>"
            )
        else:
            material_block = (
                f"          <ambient>{color}</ambient>\n"
                f"          <diffuse>{color}</diffuse>"
            )
        blades_xml.append(
            f"""
      <visual name='leaf_{i}_panel'>
        <pose>0 {yy:.6f} {z:.6f} 0 0 0</pose>
        <geometry><box><size>{0.01 * s} {2 * leaf_half_size} {2 * leaf_half_size}</size></box></geometry>
        <material>
{material_block}
        </material>
      </visual>
"""
        )
    blades = "\n".join(blades_xml)
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='link'>
      <kinematic>true</kinematic>
{blades}
    </link>
  </model>
</sdf>
"""

def make_wheel_leaf_target_sdf(model_name: str, scale: float = 1.0, color: str = "1 1 1 1") -> str:
    s = float(scale)
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='link'>
      <kinematic>true</kinematic>
      <collision name='leaf_collision'>
        <geometry><box><size>{0.003 * s} {0.18 * s} {0.18 * s}</size></box></geometry>
      </collision>
      <visual name='leaf_visual'>
        <geometry><box><size>{0.003 * s} {0.18 * s} {0.18 * s}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def make_board_target_sdf(
    model_name: str,
    scale: float = 1.0,
    texture: str = "",
    material_name: str = "MovingBoardTexture",
    aspect_ratio: float = 5.0,
) -> str:
    """长方形靶板，可贴图。moving_scale=1 时几何尺寸约为 0.18m×0.07m"""
    s = float(scale)
    # 物理尺寸：宽(x)=0.18m，高(z)=0.07m，随 scale 等比缩放
    w = 0.18 * s
    h = 0.07 * s
    t = 0.02 * s  # 厚(y)
    if texture:
        material_block = (
            "          <script><uri>file://"
            + texture
            + f"</uri><name>{material_name}</name></script>"
        )
    else:
        material_block = (
            "          <ambient>0.9 0.9 0.9 1</ambient>\n"
            "          <diffuse>0.9 0.9 0.9 1</diffuse>"
        )
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>false</static>
    <link name='link'>
      <kinematic>true</kinematic>
      <collision name='board_collision'>
        <geometry><box><size>{w} {t} {h}</size></box></geometry>
      </collision>
      <visual name='board_visual'>
        <geometry><box><size>{w} {t} {h}</size></box></geometry>
        <material>
{material_block}
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class RotatingModel:
    def __init__(self, name, base_pose, omega, start_wall):
        self.name = name
        self.base_pose = base_pose
        self.omega = omega
        self.start_wall = start_wall


class MovingBoard:
    """长方形靶板，匀速左右横移。
    move_range=从中心到一侧的距离(米)，即单侧幅度，总移动范围=2*move_range
    move_speed=平移速度(米/秒)
    """
    def __init__(self, name, base_pose, axis, move_range, move_speed, start_wall):
        self.name = name
        self.base_pose = base_pose
        self.axis = axis
        self.half_range = float(move_range)  # 从中心到一侧的距离(单侧幅度)
        speed = max(float(move_speed), 0.01)
        total_range = 2.0 * self.half_range  # 总来回距离
        self.cycle_time = total_range / speed  # 一个完整来回的周期(秒)
        self.start_wall = start_wall
        # 保存原始中心位置，避免 pose 引用共享导致位置累积
        self.origin_x = float(base_pose.position.x)
        self.origin_y = float(base_pose.position.y)
        self.origin_z = float(base_pose.position.z)


class TargetManager:
    def __init__(self):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.rate_hz = float(_get_param("update_rate", 50.0))
        self.spawn_start_delay = float(_get_param("spawn_start_delay", 2.0))

        # Overall scales (shrink default sizes)
        self.fixed_scale = float(_get_param("fixed_scale", 0.6))
        self.wheel_scale = float(_get_param("wheel_scale", 0.7))
        self.moving_scale = float(_get_param("moving_scale", 0.6))

        # 射击高度：由 shoot_sim.launch 的 shoot_height 参数设置，靶子中心对齐此高度
        self.shoot_height = float(_get_global_param("/shoot_height", 0.26))
        rospy.loginfo("[靶子高度] shoot_height=%.3fm，三个靶子中心已对齐", self.shoot_height)
        # 两套 pose：camouflage（0.8m 系数×1）、gray（1m 系数×2），由 spawn_task_board 消息选择
        def _poses(cam_key, gray_key, fallback_key, default):
            fallback = rospy.get_param(fallback_key, None) if fallback_key and not getattr(sys, "frozen", False) else None
            if fallback is not None:
                p = _pose_param(fallback_key, default)
                return p, p
            return _pose_param(cam_key, default), _pose_param(gray_key, default)

        # 1号固定靶：位置固定，用 fixed_target_pose
        self._fixed_pose = _pose_param("~fixed_target_pose", [3.0, 0.0, 0.8, 0.0])
        self._fixed_pose[2] = self.shoot_height
        self._wheel_cam, self._wheel_gray = _poses("~wheel_target_pose_camouflage", "~wheel_target_pose_gray", "~wheel_target_pose", [6.0, 0.0, 1.0, 0.0])
        self._move_cam, self._move_gray = _poses("~moving_target_pose_camouflage", "~moving_target_pose_gray", "~moving_target_pose", [2.8, 3.2, 0.2, 0.0])
        for p in (self._wheel_cam, self._wheel_gray, self._move_cam, self._move_gray):
            p[2] = self.shoot_height
        # 默认用参数，收到 spawn 消息时按消息覆盖
        self.target_2_position = (_get_param("target_2_position", "camouflage") or "camouflage").lower()
        self.target_3_position = (_get_param("target_3_position", "camouflage") or "camouflage").lower()
        self._update_poses_from_positions()

        self.fixed_name = _get_param("fixed_target_name", "target_fixed")
        self.wheel_name = _get_param("wheel_target_name", "target_wheel")

        self.moving_board_name = _get_param("moving_target_name", "target_moving")

        self.hit_mode = _get_param("hit_mode", "disappear")  # disappear|color
        self.hit_eps = float(_get_param("hit_eps", 0.02))  # meters, for fixed target plane thickness
        self.hit_cooldown = float(_get_param("hit_cooldown", 0.2))

        self.wheel_axis = _get_param("wheel_axis", "x")  # x/y/z
        self.wheel_omega = float(_get_param("wheel_omega", 0.6))  # rad/s
        self.move_axis = _get_param("move_axis", "x")  # x 或 y
        self.move_range = float(_get_param("move_range", 0.15))  # 从中心到一侧的距离(米)，总范围=2倍
        self.move_speed = float(_get_param("move_speed", 0.1))   # 平移速度(米/秒)

        self._rotating = []
        self._moving_board = None  # MovingBoard
        self._hit_models = set()
        self._seen_bullets = {}
        self._poses = {}  # model_name -> Pose
        self._missing_models = set()
        self._wheel_pose = None

        # 旋转靶 / 平移靶 正方形 AR 贴图路径（可选）
        self.wheel_textures = [
            _resolve_texture_path(_get_param("wheel_leaf_1_texture", "")),
            _resolve_texture_path(_get_param("wheel_leaf_2_texture", "")),
            _resolve_texture_path(_get_param("wheel_leaf_3_texture", "")),
            _resolve_texture_path(_get_param("wheel_leaf_4_texture", "")),
            _resolve_texture_path(_get_param("wheel_leaf_5_texture", "")),
        ]
        self.moving_board_texture = _resolve_texture_path(_get_param("moving_board_texture", ""))
        # 从贴图读取宽高比，板子尺寸适配图片比例
        self.moving_aspect = _get_image_aspect_ratio(self.moving_board_texture)
        rospy.loginfo("[板子] 平移靶贴图比例=%.2f", self.moving_aspect)
        self.fixed_target_texture = _resolve_texture_path(_get_param("fixed_target_texture", ""))

        self.hit_pub = rospy.Publisher("~hit", String, queue_size=50)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=10)

        # 是否等待 /shoot_race/spawn_task_board 后再 spawn 靶子（比赛开始时由参赛方发布）
        self.spawn_wait_for_signal = _get_param("spawn_wait_for_signal", True)
        self._targets_spawned = False

        if self.spawn_wait_for_signal:
            rospy.Subscriber("/shoot_race/spawn_task_board", String, self._on_spawn_signal, queue_size=1)
            rospy.loginfo("[靶子] 等待 /shoot_race/spawn_task_board，格式: target2,target3[,wheel_target] (c/g)")
        else:
            rospy.sleep(max(self.spawn_start_delay, 0.0))
            self._do_spawn_targets()

        # 用墙钟线程驱动旋转/平移靶，避免 use_sim_time 导致 Timer 不触发
        self._tick_stop = threading.Event()
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._tick_thread.start()
        rospy.on_shutdown(lambda: self._tick_stop.set())
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_tick_hits)

    def _update_poses_from_positions(self):
        """根据 target_X_position 更新 wheel_pose、move_pose（固定靶位置固定）"""
        self.fixed_pose = self._fixed_pose
        self.wheel_pose = self._wheel_cam if self.target_2_position == "camouflage" else self._wheel_gray
        self.move_pose = self._move_cam if self.target_3_position == "camouflage" else self._move_gray

    def _parse_positions(self, msg_data: str) -> tuple:
        """解析 spawn 消息，返回 (p2, p3)。格式: target2,target3,wheel_target，如 c,g,3"""
        s = (msg_data or "").strip()

        def _norm(s: str) -> str:
            s = s.strip().lower()
            if s in ("c", "camouflage", "迷彩"):
                return "camouflage"
            if s in ("g", "gray", "grey", "灰色"):
                return "gray"
            return "camouflage"

        if not s:
            return None, None
        parts = [p.strip() for p in s.split(",")]
        if len(parts) >= 2:
            return _norm(parts[0]), _norm(parts[1])
        if len(parts) == 1 and _norm(parts[0]) in ("camouflage", "gray"):
            return _norm(parts[0]), _norm(parts[0])
        return None, None

    def _on_spawn_signal(self, msg: String):
        """收到比赛开始信号后生成靶子。消息携带靶子位置选择（1号固定靶位置固定）"""
        if self._targets_spawned:
            rospy.loginfo("[靶子] 已生成过，忽略重复信号")
            return
        p2, p3 = self._parse_positions(msg.data if msg else "")
        if p2 is not None:
            self.target_2_position, self.target_3_position = p2, p3
            self._update_poses_from_positions()
            rospy.loginfo("[靶子] 位置选择: 2=%s 3=%s", p2, p3)
        rospy.sleep(max(self.spawn_start_delay, 0.0))
        self._do_spawn_targets()
        self._targets_spawned = True

    def _do_spawn_targets(self):
        """按 target_X_position 选择的位置生成三个靶子"""
        if _model_exists(self.get_state_srv, "target_static_board"):
            try:
                self.delete_srv("target_static_board")
                rospy.sleep(0.1)
            except Exception:
                pass
        _spawn_sdf(
            self.spawn_srv,
            self.delete_srv,
            self.get_state_srv,
            self.fixed_name,
            make_fixed_ring_target_sdf(self.fixed_name, scale=self.fixed_scale, texture=self.fixed_target_texture),
            _pose(*self.fixed_pose),
            retries=30,
            delay_s=0.5,
        )
        rospy.sleep(0.2)
        _spawn_sdf(
            self.spawn_srv,
            self.delete_srv,
            self.get_state_srv,
            self.wheel_name,
            make_ferris_wheel_target_sdf(self.wheel_name, 5, scale=self.wheel_scale, textures=self.wheel_textures),
            _pose(*self.wheel_pose),
        )
        rospy.sleep(0.2)
        now = time.time()
        self._rotating.append(RotatingModel(self.wheel_name, _pose(*self.wheel_pose), self.wheel_omega, now))
        s = self.moving_scale
        board_pose = _pose(*self.move_pose)
        rospy.loginfo(
            "[靶子] 已生成 固定靶(固定) 旋转靶(%s) 平移靶(%s)",
            self.target_2_position, self.target_3_position
        )
        rospy.loginfo(
            "[平移靶] move_range=%.2fm(单侧) 总范围=%.2fm move_speed=%.2fm/s 轴=%s",
            self.move_range, 2.0 * self.move_range, self.move_speed, self.move_axis
        )
        # 必须先 spawn 再设置 _moving_board，否则 _tick_loop 可能在模型未就绪时调用 SetModelState 失败并加入 _missing_models，导致永不更新
        _spawn_sdf(
            self.spawn_srv,
            self.delete_srv,
            self.get_state_srv,
            self.moving_board_name,
            make_board_target_sdf(
                self.moving_board_name,
                scale=s,
                texture=self.moving_board_texture,
                material_name="MovingBoardTexture",
                aspect_ratio=self.moving_aspect,
            ),
            board_pose,
        )
        self._moving_board = MovingBoard(
            self.moving_board_name,
            board_pose, self.move_axis, self.move_range, self.move_speed, now
        )

    def _tick_loop(self):
        """墙钟驱动：旋转靶、平移靶位置更新，不受 use_sim_time 影响"""
        period = 1.0 / self.rate_hz
        while not self._tick_stop.wait(timeout=period):
            try:
                now = time.time()
                for m in self._rotating:
                    t = now - m.start_wall
                    ang = m.omega * t
                    pose = Pose()
                    pose.position = m.base_pose.position
                    base_yaw = 2.0 * math.atan2(m.base_pose.orientation.z, m.base_pose.orientation.w)
                    axis = (self.wheel_axis or "x").lower().strip()
                    if axis == "x":
                        pose.orientation = _quat_from_rpy(ang, 0.0, base_yaw)
                    elif axis == "y":
                        pose.orientation = _quat_from_rpy(0.0, ang, base_yaw)
                    else:
                        pose.orientation = _quat_from_rpy(0.0, 0.0, base_yaw + ang)
                    self._set_pose(m.name, pose)
                if self._moving_board:
                    m = self._moving_board
                    t = now - m.start_wall
                    phase = (t % max(m.cycle_time, 0.01)) / max(m.cycle_time, 0.01)
                    if phase < 0.5:
                        delta = -m.half_range + 2.0 * m.half_range * (phase / 0.5)
                    else:
                        delta = m.half_range - 2.0 * m.half_range * ((phase - 0.5) / 0.5)
                    pose = Pose()
                    pose.position.x = m.origin_x + (delta if m.axis == "x" else 0.0)
                    pose.position.y = m.origin_y + (delta if m.axis == "y" else 0.0)
                    pose.position.z = m.origin_z
                    pose.orientation = m.base_pose.orientation
                    self._set_pose(m.name, pose)
            except rospy.ROSInterruptException:
                break
            except Exception:
                pass

    def _on_tick_hits(self, _evt):
        """仅做命中检测，旋转/平移由 _tick_loop 驱动"""
        self._check_hits()

    def _set_pose(self, model_name: str, pose: Pose, ref_frame: str = "world"):
        if model_name in self._missing_models:
            return
        st = ModelState()
        st.model_name = model_name
        st.pose = pose
        st.twist = Twist()
        st.twist.linear = Vector3(0.0, 0.0, 0.0)
        st.twist.angular = Vector3(0.0, 0.0, 0.0)
        st.reference_frame = ref_frame
        req = SetModelStateRequest()
        req.model_state = st
        try:
            resp = self.set_state_srv(req)
            if hasattr(resp, "success") and not resp.success:
                # 如果 Gazebo 报模型不存在，就记下来以后不再更新它，避免持续刷错误
                if "does not exist" in getattr(resp, "status_message", ""):
                    self._missing_models.add(model_name)
        except Exception:
            pass

    def _on_model_states(self, msg: ModelStates):
        # Cache latest poses for fast geometric hit tests
        # 先记录当前消息里有哪些模型，用来清理已经从 Gazebo 删除的子弹
        current = set(msg.name)
        # 删除已经不存在的 bullet_XXX，避免“幽灵子弹”反复触发命中与 DeleteModel 错误
        for name in list(self._poses.keys()):
            if name.startswith("bullet_") and name not in current:
                self._poses.pop(name, None)
                self._seen_bullets.pop(name, None)

        for name, pose in zip(msg.name, msg.pose):
            self._poses[name] = pose
            if name == self.wheel_name:
                self._wheel_pose = pose

    def _check_hits(self):
        now = rospy.Time.now().to_sec()
        # Iterate bullets (model names start with bullet_)
        for bname, bpose in list(self._poses.items()):
            if not bname.startswith("bullet_"):
                continue
            last = self._seen_bullets.get(bname, 0.0)
            if now - last < self.hit_cooldown:
                continue

            # Fixed target ring score: near plane and within radius
            score = self._score_fixed(bpose.position.x, bpose.position.y, bpose.position.z)
            if score > 0:
                self._seen_bullets[bname] = now
                self._delete_best_effort(bname)
                # 命中后立刻从缓存移除，防止重复判定
                self._poses.pop(bname, None)
                self.hit_pub.publish(String(data=f"fixed score={score} bullet={bname}"))
                continue

            # Wheel: compute which sector (1-5) bullet hits, if any
            leaf_idx = self._score_wheel(bpose.position.x, bpose.position.y, bpose.position.z)
            if leaf_idx > 0:
                self._seen_bullets[bname] = now
                self._delete_best_effort(bname)
                self._poses.pop(bname, None)
                self.hit_pub.publish(String(data=f"wheel leaf={leaf_idx} bullet={bname}"))
                continue

            # 第三个靶子：平移长方形靶板，分 3 个区域判定
            region = self._score_board(bpose.position.x, bpose.position.y, bpose.position.z, self.moving_board_name)
            if region > 0:
                self._seen_bullets[bname] = now
                self._delete_best_effort(bname)
                self._poses.pop(bname, None)
                self.hit_pub.publish(String(data=f"moving region={region} bullet={bname}"))

    def _delete_best_effort(self, model_name: str):
        try:
            self.delete_srv(model_name)
        except Exception:
            pass

    def _hit_box_any(self, bullet_pose: Pose, target_names, half):
        bx, by, bz = bullet_pose.position.x, bullet_pose.position.y, bullet_pose.position.z
        hx, hy, hz = half
        for n in target_names:
            if n in self._hit_models:
                continue
            tp = self._poses.get(n)
            if not tp:
                continue
            dx = bx - tp.position.x
            dy = by - tp.position.y
            dz = bz - tp.position.z
            if abs(dx) <= hx and abs(dy) <= hy and abs(dz) <= hz:
                return n
        return None

    def _hit_cyl_any(self, bullet_pose: Pose, target_names, radius: float, half_height: float):
        bx, by, bz = bullet_pose.position.x, bullet_pose.position.y, bullet_pose.position.z
        for n in target_names:
            if n in self._hit_models:
                continue
            tp = self._poses.get(n)
            if not tp:
                continue
            dx = bx - tp.position.x
            dy = by - tp.position.y
            dz = bz - tp.position.z
            if abs(dz) > (half_height + 0.05):
                continue
            if dx * dx + dy * dy <= (radius + 0.02) ** 2:
                return n
        return None

    def _score_fixed(self, wx: float, wy: float, wz: float) -> int:
        # Convert point world->target local;若靠近靶面所在平面，再按半径计算环数
        try:
            # Get model state via service (cheap enough for hits)
            from gazebo_msgs.srv import GetModelState

            if not hasattr(self, "_get_state"):
                rospy.wait_for_service("/gazebo/get_model_state")
                self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            resp = self._get_state(self.fixed_name, "world")
            if not resp.success:
                return 0
            px, py, pz = resp.pose.position.x, resp.pose.position.y, resp.pose.position.z
            q = resp.pose.orientation
        except Exception:
            return 0

        # Inverse rotate (general quaternion inverse)
        x, y, z, w = q.x, q.y, q.z, q.w
        # normalize
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-9:
            return 0
        x, y, z, w = x / n, y / n, z / n, w / n
        # inverse quaternion
        xi, yi, zi, wi = -x, -y, -z, w

        vx, vy, vz = wx - px, wy - py, wz - pz
        # q^{-1} * v * q (treat v as pure quaternion)
        # first: qinv * v
        rx = wi * vx + yi * vz - zi * vy
        ry = wi * vy + zi * vx - xi * vz
        rz = wi * vz + xi * vy - yi * vx
        rw = -xi * vx - yi * vy - zi * vz
        # then: (qinv*v) * q，得到在靶子局部坐标系下的点
        lx = rw * x + rx * w + ry * z - rz * y  # 法向(板厚方向，大致是局部 x)
        ly = rw * y - rx * z + ry * w + rz * x  # 局部 y
        lz = rw * z + rx * y - ry * x + rz * w  # 局部 z

        # 子弹需在靶面附近：容差需覆盖子弹每帧位移，否则高速子弹会“穿过”而漏检
        # 30m/s @ 100Hz → 每帧 0.3m，故深度容差至少 0.15m
        hit_depth = max(self.hit_eps, 0.15)
        if abs(lx) > hit_depth:
            return 0

        # 标准环靶计分：十环/九环/... 的数值是直径(cm)，需要换算为半径(m)
        # 十环3cm、九环9cm、八环15cm、七环21cm、六环27cm（均按 fixed_scale 等比缩放）
        r = math.sqrt(ly * ly + lz * lz)
        s = self.fixed_scale
        r10 = 0.015 * s  # 3cm 直径 -> 1.5cm 半径
        r9  = 0.045 * s  # 9cm 直径 -> 4.5cm 半径
        r8  = 0.075 * s  # 15cm -> 7.5cm
        r7  = 0.105 * s  # 21cm -> 10.5cm
        r6  = 0.135 * s  # 27cm -> 13.5cm
        if r <= r10:
            return 10  # 十环
        if r <= r9:
            return 9   # 九环
        if r <= r8:
            return 8   # 八环
        if r <= r7:
            return 7   # 七环
        if r <= r6:
            return 6   # 六环
        return 0

    def _score_wheel(self, wx: float, wy: float, wz: float) -> int:
        """Return leaf index 1-5 if bullet hits a small square leaf, else 0."""
        pose = self._wheel_pose or self._poses.get(self.wheel_name)
        if not pose:
            return 0
        px, py, pz = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation

        # world -> wheel local
        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-9:
            return 0
        x, y, z, w = x / n, y / n, z / n, w / n
        xi, yi, zi, wi = -x, -y, -z, w

        vx, vy, vz = wx - px, wy - py, wz - pz
        rx = wi * vx + yi * vz - zi * vy
        ry = wi * vy + zi * vx - xi * vz
        rz = wi * vz + xi * vy - yi * vx
        rw = -xi * vx - yi * vy - zi * vz
        lx = rw * x + rx * w + ry * z - rz * y
        ly = rw * y - rx * z + ry * w + rz * x
        lz = rw * z + rx * y - ry * x + rz * w

        # 与 make_ferris_wheel_target_sdf 中的小正方形几何保持一致：
        # wheel_scale=1 时，小正方形边长约 0.05m，距旋转中心约 0.09m。
        # 为了补偿子弹半径和视觉误差，这里稍微放宽一点命中盒的范围。
        s = self.wheel_scale
        leaf_half_size = 0.025 * s
        leaf_radius = 0.09 * s
        # 深度容差需覆盖子弹每帧位移(30m/s@100Hz≈0.3m)，否则高速子弹会漏检
        half_thickness = max(0.008 * s, 0.15)

        # 轮子平面近似在 YZ 平面，围绕 X 轴旋转。逐个检查子弹是否进入某个小正方形的包围盒
        for i in range(5):
            ang = 2.0 * math.pi * i / 5.0
            cy = leaf_radius * math.sin(ang)
            cz = leaf_radius * math.cos(ang)
            if (
                abs(lx) <= half_thickness
                and abs(ly - cy) <= 1.1 * leaf_half_size
                and abs(lz - cz) <= 1.1 * leaf_half_size
            ):
                return i + 1
        return 0

    def _score_board(self, wx: float, wy: float, wz: float, board_name: str) -> int:
        """长方形靶板分 3 个区域，返回 1/2/3 或 0。板子尺寸与 make_board_target_sdf 一致"""
        pose = self._poses.get(board_name)
        if not pose:
            return 0
        px, py, pz = pose.position.x, pose.position.y, pose.position.z
        q = pose.orientation

        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-9:
            return 0
        x, y, z, w = x / n, y / n, z / n, w / n
        xi, yi, zi, wi = -x, -y, -z, w

        vx, vy, vz = wx - px, wy - py, wz - pz
        rx = wi * vx + yi * vz - zi * vy
        ry = wi * vy + zi * vx - xi * vz
        rz = wi * vz + xi * vy - yi * vx
        rw = -xi * vx - yi * vy - zi * vz
        lx = rw * x + rx * w + ry * z - rz * y
        ly = rw * y - rx * z + ry * w + rz * x
        lz = rw * z + rx * y - ry * x + rz * w

        s = self.moving_scale
        # 与 make_board_target_sdf 保持一致：moving_scale=1 时板子约 0.18m×0.07m
        base_w = 0.18 * s
        base_h = 0.07 * s
        # 厚度方向容差需覆盖子弹每帧位移(30m/s@100Hz)，板子朝向不定故三个方向都放宽
        depth_margin = 0.15
        hw = 0.5 * base_w + 0.02 + depth_margin
        ht = 0.01 * s + 0.02 + depth_margin
        hh = 0.5 * base_h + 0.02 + depth_margin
        if abs(lx) > hw or abs(ly) > ht or abs(lz) > hh:
            return 0
        # 沿 x 分 3 区：左 1，中 2，右 3（按比例）
        if lx < -hw / 3:
            return 1
        if lx <= hw / 3:
            return 2
        return 3


def main():
    rospy.init_node("shoot_targets")
    _ = TargetManager()
    rospy.spin()


if __name__ == "__main__":
    main()

