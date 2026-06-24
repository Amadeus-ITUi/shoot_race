/*
 * Copyright 2013 Open Source Robotics Foundation
 * Modified: Preserve Z velocity and angular X/Y for proper gravity and ramp handling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef GAZEBO_ROS_PLANAR_MOVE_FIXED_HH
#define GAZEBO_ROS_PLANAR_MOVE_FIXED_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

  class GazeboRosPlanarMoveFixed : public ModelPlugin {

    public:
      GazeboRosPlanarMoveFixed();
      ~GazeboRosPlanarMoveFixed();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher odometry_pub_;
      ros::Subscriber vel_sub_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      double odometry_rate_;
      double cmd_timeout_;
      ros::Time last_cmd_received_time_;

      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_;
      double y_;
      double rot_;
      bool alive_;
      common::Time last_odom_publish_time_;
      ignition::math::Pose3d last_odom_pose_;
      bool stationary_yaw_locked_;
      double locked_yaw_;

  };

}

#endif
