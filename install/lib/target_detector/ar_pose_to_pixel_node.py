#!/usr/bin/env python3
"""
AR 码位姿转像素坐标：订阅 ar_track_alvar 的 /ar_pose_marker，结合 camera_info 投影到像素坐标
输出: /ar_marker_pixels (MarkerPixelArray)
"""
import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import CameraInfo
from target_detector.msg import MarkerPixelArray, MarkerPixel


class ArPoseToPixelNode:
    def __init__(self):
        rospy.init_node("ar_pose_to_pixel", anonymous=False)

        self._ar_topic = rospy.get_param("~ar_pose_topic", "/ar_pose_marker")
        self._info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
        self._output_topic = rospy.get_param("~output_topic", "/ar_marker_pixels")

        self._K = None  # 3x3 相机内参
        self._sub_ar = rospy.Subscriber(self._ar_topic, AlvarMarkers, self._on_ar, queue_size=10)
        self._sub_info = rospy.Subscriber(self._info_topic, CameraInfo, self._on_info, queue_size=1)
        self._pub = rospy.Publisher(self._output_topic, MarkerPixelArray, queue_size=10)

        rospy.loginfo("[ar_pose_to_pixel] 订阅 %s, %s | 发布 %s",
                      self._ar_topic, self._info_topic, self._output_topic)

    def _on_info(self, msg):
        self._K = np.array(msg.K, dtype=np.float64).reshape(3, 3)

    def _project_to_pixel(self, x, y, z):
        """将相机坐标系下的 (x,y,z) 投影到像素 (u,v)"""
        if self._K is None or abs(z) < 1e-6:
            return None, None
        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]
        u = fx * x / z + cx
        v = fy * y / z + cy
        return float(u), float(v)

    def _on_ar(self, msg):
        if self._K is None:
            return
        out = MarkerPixelArray()
        out.header = msg.header
        for m in msg.markers:
            x = m.pose.pose.position.x
            y = m.pose.pose.position.y
            z = m.pose.pose.position.z
            u, v = self._project_to_pixel(x, y, z)
            if u is not None:
                mp = MarkerPixel()
                mp.id = m.id
                mp.u = u
                mp.v = v
                out.markers.append(mp)
        if out.markers:
            self._pub.publish(out)

    def run(self):
        rospy.spin()


def main():
    n = ArPoseToPixelNode()
    n.run()


if __name__ == "__main__":
    main()
