#!/bin/bash
# 修复 ros-noetic-hector-pose-estimation 缺失的 nodelets.xml（apt 包打包遗漏）
# 需 sudo 执行: sudo bash fix_hector_pose_estimation.sh

XML_PATH="/opt/ros/noetic/share/hector_pose_estimation/hector_pose_estimation_nodelets.xml"

cat > "$XML_PATH" << 'EOF'
<library path="lib/libhector_pose_estimation_nodelet">
  <class name="hector_pose_estimation/PoseEstimationNodelet" type="hector_pose_estimation::PoseEstimationNodelet" base_class_type="nodelet::Nodelet">
    <description>This nodelet initializes the pose estimation filter with a generic system model driven by IMU measurements only.</description>
  </class>
</library>
EOF

echo "已创建 $XML_PATH"
ls -la "$XML_PATH"
