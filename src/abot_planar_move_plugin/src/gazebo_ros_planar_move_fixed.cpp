/*
 * Copyright 2013 Open Source Robotics Foundation
 * Modified: Preserve Z velocity and angular X/Y for proper gravity and ramp handling
 * Fix for: https://github.com/ros-simulation/gazebo_ros_pkgs/issues/121
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "abot_planar_move_plugin/gazebo_ros_planar_move_fixed.h"
#include <ignition/math/Vector3.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>

namespace gazebo {

GazeboRosPlanarMoveFixed::GazeboRosPlanarMoveFixed() {}

GazeboRosPlanarMoveFixed::~GazeboRosPlanarMoveFixed() {}

void GazeboRosPlanarMoveFixed::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  parent_ = parent;

  robot_namespace_ = "";
  if (sdf->HasElement("robotNamespace"))
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();

  command_topic_ = "cmd_vel";
  if (sdf->HasElement("commandTopic"))
    command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();

  odometry_topic_ = "odom";
  if (sdf->HasElement("odometryTopic"))
    odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();

  odometry_frame_ = "odom";
  if (sdf->HasElement("odometryFrame"))
    odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();

  robot_base_frame_ = "base_footprint";
  if (sdf->HasElement("robotBaseFrame"))
    robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();

  odometry_rate_ = 20.0;
  if (sdf->HasElement("odometryRate"))
    odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();

  cmd_timeout_ = -1;
  if (sdf->HasElement("cmdTimeout"))
    cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();

#if GAZEBO_MAJOR_VERSION >= 8
  last_odom_publish_time_ = parent_->GetWorld()->SimTime();
  last_odom_pose_ = parent_->WorldPose();
#else
  last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
  last_odom_pose_ = parent_->GetWorldPose().Ign();
#endif

  x_ = 0;
  y_ = 0;
  rot_ = 0;
  alive_ = true;
  stationary_yaw_locked_ = false;
  locked_yaw_ = 0;

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("planar_move_fixed", "PlanarMovePlugin (ns = " << robot_namespace_
      << "). A ROS node for Gazebo has not been initialized.");
    return;
  }
  rosnode_.reset(new ros::NodeHandle(robot_namespace_));

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_.reset(new tf::TransformBroadcaster());

  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
        boost::bind(&GazeboRosPlanarMoveFixed::cmdVelCallback, this, _1),
        ros::VoidPtr(), &queue_);

  vel_sub_ = rosnode_->subscribe(so);
  odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

  callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosPlanarMoveFixed::QueueThread, this));

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosPlanarMoveFixed::UpdateChild, this));
}

void GazeboRosPlanarMoveFixed::UpdateChild()
{
  boost::mutex::scoped_lock scoped_lock(lock);
  if (cmd_timeout_ >= 0)
  {
    if ((ros::Time::now() - last_cmd_received_time_).toSec() > cmd_timeout_)
    {
      x_ = 0;
      y_ = 0;
      rot_ = 0;
    }
  }

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d pose = parent_->WorldPose();
  double linear_vel_z = parent_->WorldLinearVel().Z();
  double angular_vel_x = parent_->WorldAngularVel().X();
  double angular_vel_y = parent_->WorldAngularVel().Y();
#else
  ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
  double linear_vel_z = parent_->GetWorldLinearVel().Z();
  double angular_vel_x = parent_->GetWorldAngularVel().X();
  double angular_vel_y = parent_->GetWorldAngularVel().Y();
#endif

  bool stationary = (x_ == 0 && y_ == 0 && rot_ == 0);

  if (stationary) {
    if (!stationary_yaw_locked_) {
      stationary_yaw_locked_ = true;
      locked_yaw_ = pose.Rot().Yaw();
    }
    // 静止时强制姿态：锁 yaw 防漂移，保留 roll/pitch 以应对斜坡
    double roll = pose.Rot().Roll();
    double pitch = pose.Rot().Pitch();
    ignition::math::Quaterniond q;
    q.Euler(roll, pitch, locked_yaw_);
    parent_->SetWorldPose(ignition::math::Pose3d(pose.Pos(), q));
  } else {
    stationary_yaw_locked_ = false;
  }

  float yaw = pose.Rot().Yaw();
  parent_->SetLinearVel(ignition::math::Vector3d(
      x_ * cosf(yaw) - y_ * sinf(yaw),
      y_ * cosf(yaw) + x_ * sinf(yaw),
      linear_vel_z));

  double ax = 0, ay = 0, az = rot_;
  if (!stationary) {
    ax = angular_vel_x;
    ay = angular_vel_y;
  }
  parent_->SetAngularVel(ignition::math::Vector3d(ax, ay, az));

  if (odometry_rate_ > 0.0)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent_->GetWorld()->SimTime();
#else
    common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = (current_time - last_odom_publish_time_).Double();
    if (seconds_since_last_update > (1.0 / odometry_rate_))
    {
      publishOdometry(seconds_since_last_update);
      last_odom_publish_time_ = current_time;
    }
  }
}

void GazeboRosPlanarMoveFixed::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

void GazeboRosPlanarMoveFixed::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  last_cmd_received_time_ = ros::Time::now();
  x_ = cmd_msg->linear.x;
  y_ = cmd_msg->linear.y;
  rot_ = cmd_msg->angular.z;
}

void GazeboRosPlanarMoveFixed::QueueThread()
{
  static const double timeout = 0.01;
  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosPlanarMoveFixed::publishOdometry(double step_time)
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
  std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d pose = parent_->WorldPose();
#else
  ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
#endif

  tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(
      tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));

  odom_.pose.pose.position.x = pose.Pos().X();
  odom_.pose.pose.position.y = pose.Pos().Y();
  odom_.pose.pose.orientation.x = pose.Rot().X();
  odom_.pose.pose.orientation.y = pose.Rot().Y();
  odom_.pose.pose.orientation.z = pose.Rot().Z();
  odom_.pose.pose.orientation.w = pose.Rot().W();
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  ignition::math::Vector3d linear;
  linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
  linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
  if (rot_ > M_PI / step_time)
    odom_.twist.twist.angular.z = rot_;
  else
  {
    float last_yaw = last_odom_pose_.Rot().Yaw();
    float current_yaw = pose.Rot().Yaw();
    while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
    while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
    odom_.twist.twist.angular.z = (current_yaw - last_yaw) / step_time;
  }
  last_odom_pose_ = pose;

  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  odometry_pub_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlanarMoveFixed)

}
