#!/usr/bin/env python3
import math
import sys
import threading
import uuid

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
import tf2_ros
from geometry_msgs.msg import Vector3, Pose, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState


def _quat_to_rotmat(q: Quaternion):
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _unit_axis(axis_name: str):
    a = (axis_name or "x").lower().strip()
    if a == "x":
        return (1.0, 0.0, 0.0)
    if a == "y":
        return (0.0, 1.0, 0.0)
    if a == "z":
        return (0.0, 0.0, 1.0)
    raise ValueError("shoot_axis must be one of: x, y, z")


def _mat_vec(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def make_sphere_sdf(model_name: str, radius: float, mass: float, mu: float, mu2: float, restitution: float):
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>false</static>
    <link name='link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{0.4 * mass * radius * radius}</ixx>
          <iyy>{0.4 * mass * radius * radius}</iyy>
          <izz>{0.4 * mass * radius * radius}</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu2}</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>{restitution}</restitution_coefficient>
            <threshold>1e-3</threshold>
          </bounce>
          <contact>
            <ode/>
          </contact>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <material>
          <ambient>1 0.2 0.2 1</ambient>
          <diffuse>1 0.2 0.2 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ShooterNode:
    def __init__(self):
        # Frame used ONLY for TF lookup (usually 'odom' in this sim).
        # Gazebo's spawn/set services use the Gazebo "world" frame regardless of TF naming.
        self.world_frame = _get_param("world_frame", "odom")
        self.shoot_frame = _get_param("shoot_frame", "shout_link")
        self.shoot_axis = _get_param("shoot_axis", "x")
        self.bullet_radius = float(_get_param("bullet_radius", 0.03))
        self.bullet_mass = float(_get_param("bullet_mass", 0.05))
        self.bullet_speed = float(_get_param("bullet_speed", 15.0))
        self.spawn_offset = float(_get_param("spawn_offset", 0.05))
        self.bullet_lifetime = float(_get_param("bullet_lifetime", 6.0))
        self.friction_mu = float(_get_param("friction_mu", 0.6))
        self.friction_mu2 = float(_get_param("friction_mu2", 0.6))
        self.restitution = float(_get_param("restitution", 0.05))
        # 与靶子共用的射击高度参数（/shoot_height），用于统一子弹发射高度
        self.shoot_height = float(_get_global_param("/shoot_height", 0.26))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/delete_model")
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self._lock = threading.Lock()
        rospy.Service("~shoot", Trigger, self.on_shoot)

    def on_shoot(self, _req):
        with self._lock:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.world_frame, self.shoot_frame, rospy.Time(0), rospy.Duration(0.3)
                )
            except Exception as e:
                return TriggerResponse(success=False, message=f"TF lookup failed: {e}")

            q = t.transform.rotation
            p = t.transform.translation
            try:
                axis_local = _unit_axis(self.shoot_axis)
            except Exception as e:
                return TriggerResponse(success=False, message=str(e))

            rot = _quat_to_rotmat(q)
            axis_world = _mat_vec(rot, axis_local)
            n = _norm(axis_world)
            if n < 1e-9:
                return TriggerResponse(success=False, message="Invalid shoot axis (near zero).")
            axis_world = _scale(axis_world, 1.0 / n)

            # 先按 TF 方向计算偏移，再强制使用统一的射击高度 shoot_height
            spawn_pos = _add(
                (p.x, p.y, p.z),
                _scale(axis_world, self.spawn_offset + self.bullet_radius),
            )
            spawn_pos = (spawn_pos[0], spawn_pos[1], self.shoot_height)

            model_name = f"bullet_{uuid.uuid4().hex[:10]}"
            sdf_xml = make_sphere_sdf(
                model_name=model_name,
                radius=self.bullet_radius,
                mass=self.bullet_mass,
                mu=self.friction_mu,
                mu2=self.friction_mu2,
                restitution=self.restitution,
            )

            spawn_req = SpawnModelRequest()
            spawn_req.model_name = model_name
            spawn_req.model_xml = sdf_xml
            spawn_req.robot_namespace = rospy.get_namespace()
            # SpawnModel's reference_frame must be a Gazebo frame/entity ("" == world),
            # not an arbitrary TF frame like 'odom'.
            spawn_req.reference_frame = ""
            spawn_req.initial_pose = Pose()
            spawn_req.initial_pose.position.x = spawn_pos[0]
            spawn_req.initial_pose.position.y = spawn_pos[1]
            spawn_req.initial_pose.position.z = spawn_pos[2]
            spawn_req.initial_pose.orientation = q

            try:
                resp = self.spawn_srv(spawn_req)
                if not resp.success:
                    return TriggerResponse(success=False, message=f"spawn failed: {resp.status_message}")
            except Exception as e:
                return TriggerResponse(success=False, message=f"spawn service error: {e}")

            state = ModelState()
            state.model_name = model_name
            state.pose = spawn_req.initial_pose
            state.reference_frame = "world"
            state.twist.linear = Vector3(*_scale(axis_world, self.bullet_speed))

            set_req = SetModelStateRequest()
            set_req.model_state = state
            try:
                _ = self.set_state_srv(set_req)
            except Exception as e:
                return TriggerResponse(success=False, message=f"set_state error: {e}")

            rospy.Timer(
                rospy.Duration(self.bullet_lifetime),
                lambda _evt: self._delete_best_effort(model_name),
                oneshot=True,
            )

            return TriggerResponse(success=True, message=model_name)

    def _delete_best_effort(self, model_name: str):
        try:
            self.delete_srv(model_name)
        except Exception:
            pass


def main():
    rospy.init_node("shoot_sim")
    _ = ShooterNode()
    rospy.spin()


if __name__ == "__main__":
    main()

