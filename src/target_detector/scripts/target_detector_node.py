#!/usr/bin/env python3
"""
圆环靶心检测节点：订阅相机图像，霍夫圆变换检测靶心像素坐标
输入：仿真相机图像 (sensor_msgs/Image)
输出：靶心像素坐标 (geometry_msgs/PointStamped)，x=u(列), y=v(行)
"""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped


class TargetDetectorNode:
    def __init__(self):
        rospy.init_node("target_detector", anonymous=False)
        self._bridge = CvBridge()

        # 参数
        self._image_topic = rospy.get_param("~image_topic", "/camera/image")
        self._camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
        self._output_topic = rospy.get_param("~output_topic", "/target_center_pixel")
        self._frame_id = rospy.get_param("~frame_id", "camera_link")
        self._visualize = rospy.get_param("~visualize", True)
        # 霍夫圆参数
        self._dp = rospy.get_param("~dp", 1.0)  # 分辨率累加器与图像分辨率之比
        self._min_dist = rospy.get_param("~min_dist", 50)  # 最小圆心距
        self._param1 = rospy.get_param("~param1", 100)  # Canny 高阈值 100~150 150~200
        self._param2 = rospy.get_param("~param2", 30)  # 累加器阈值
        self._min_radius = rospy.get_param("~min_radius", 10)  # 最小半径
        self._max_radius = rospy.get_param("~max_radius", 0)  # 0 表示不限制
        self._publish_rate = rospy.get_param("~publish_rate", 10)  # 有检测结果时的发布频率
        # 同心圆验证：圆心距离阈值，只有多圆同心才认为是真靶
        self._center_tol = rospy.get_param("~center_tolerance", 25)
        self._min_concentric = rospy.get_param("~min_concentric", 2)  # 至少几个圆同心
        self._smooth_alpha = rospy.get_param("~smooth_alpha", 0.3)  # 时间平滑系数，越小越平滑

        self._pub = rospy.Publisher(self._output_topic, PointStamped, queue_size=10)
        self._pub_debug = rospy.Publisher("~debug_image", Image, queue_size=2)
        self._pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=2)
        self._sub = rospy.Subscriber(self._image_topic, Image, self._on_image, queue_size=1)
        self._sub_info = rospy.Subscriber(self._camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        self._last_center = None  # (u, v)

        rospy.loginfo(
            "[target_detector] 订阅 %s、%s，发布 %s、~camera_info，可视化=%s",
            self._image_topic, self._camera_info_topic, self._output_topic, self._visualize
        )

    def _on_camera_info(self, msg: CameraInfo):
        """转发 camera_info 到 ~camera_info，供 RViz Camera 显示使用"""
        self._pub_camera_info.publish(msg)

    def _detect_target_center(self, cv_image: np.ndarray) -> tuple:
        """
        霍夫圆变换检测靶心，返回 (center, circles) 或 (None, None)
        center: (u, v) 像素坐标
        circles: [(u,v,r), ...] 所有检测到的圆，用于可视化
        """
        if cv_image is None or cv_image.size == 0:
            return None, None
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        h, w = gray.shape
        max_r = self._max_radius if self._max_radius > 0 else min(w, h) // 2

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=self._dp,
            minDist=self._min_dist,
            param1=self._param1,
            param2=self._param2,
            minRadius=self._min_radius,
            maxRadius=max_r,
        )

        if circles is None:
            return None, None

        circles = np.uint16(np.around(circles))
        # 取半径最小的圆作为靶心（同心圆中靶心是最内层）
        best = None
        circle_list = []
        for c in circles[0, :]:
            u, v, r = int(c[0]), int(c[1]), int(c[2])
            circle_list.append((u, v, r))
            if best is None or r < best[2]:
                best = (u, v, r)

        if best is None:
            return None, circle_list
        return (best[0], best[1]), circle_list

    def _draw_visualization(self, cv_image: np.ndarray, center, circles, msg: Image):
        """在图像上绘制检测结果并发布"""
        vis = cv_image.copy()
        if circles:
            for u, v, r in circles:
                cv2.circle(vis, (u, v), r, (0, 255, 0), 2)  # 绿色圆
        if center is not None:
            u, v = int(round(center[0])), int(round(center[1]))
            cv2.circle(vis, (u, v), 5, (0, 0, 255), -1)  # 红色靶心
            cv2.drawMarker(vis, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(vis, "Target", (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        debug_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header = msg.header
        self._pub_debug.publish(debug_msg)

    def _on_image(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr_throttle(5.0, "[target_detector] 图像转换失败: %s", e)
            return

        center, circles = self._detect_target_center(cv_image)
        if center is not None:
            # 时间平滑，减少抖动
            if self._last_center is not None and self._smooth_alpha < 1.0:
                u = self._smooth_alpha * self._last_center[0] + (1 - self._smooth_alpha) * center[0]
                v = self._smooth_alpha * self._last_center[1] + (1 - self._smooth_alpha) * center[1]
                center = (u, v)
            self._last_center = center
            out = PointStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self._frame_id
            out.point.x = float(center[0])
            out.point.y = float(center[1])
            out.point.z = 0.0
            self._pub.publish(out)

        if self._visualize:
            self._draw_visualization(cv_image, center, circles or [], msg)

    def run(self):
        rospy.spin()


def main():
    n = TargetDetectorNode()
    n.run()


if __name__ == "__main__":
    main()
