// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LOOPED_MISSION_PLANNER__LOOPED_MISSION_PLANNER_CORE_HPP_
#define LOOPED_MISSION_PLANNER__LOOPED_MISSION_PLANNER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_serializer.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

#include <string>

class LoopedMissionPlanner : public rclcpp::Node
{
public:
  LoopedMissionPlanner();
  ~LoopedMissionPlanner();

private:
  void loadGoalRosbag();
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr);
  void timerCallback();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_pose_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr
    engage_pub_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr
    vel_lim_pub_;
  rclcpp::TimerBase::SharedPtr timer_control_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr_;

  // geometry_msgs::msg::Pose::ConstSharedPtr point_a_pose_ptr_;
  // geometry_msgs::msg::Pose::ConstSharedPtr point_b_pose_ptr_;
  double goal_a_px_, goal_a_py_, goal_b_px_, goal_b_py_;
  double goal_a_qz_, goal_a_qw_, goal_b_qz_, goal_b_qw_;

  std::vector<geometry_msgs::msg::Pose> pose_checkpoints_;

  std::string rosbag_path_;
  double goal_tolerance_;
  bool going_to_point_a_;
  double timer_dt_;
  int lap_num_;
  int num_checkpoints_;
};

#endif  // LOOPED_MISSION_PLANNER__LOOPED_MISSION_PLANNER_CORE_HPP_
