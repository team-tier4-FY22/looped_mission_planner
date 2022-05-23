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

#include "looped_mission_planner/looped_mission_planner_core.hpp"

#include <cmath>
#include <memory>
#include <string>

LoopedMissionPlanner::LoopedMissionPlanner()
: Node("looped_mission_planner")
{
  goal_tolerance_ = 15.0;
  timer_dt_ = 0.1;
  going_to_point_a_ = true;
  lap_num_ = 0;
  num_checkpoints_ = 0;
  rosbag_path_ = declare_parameter("rosbag_path", "");

  goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("out_goal_pose", rclcpp::QoS{10});
  engage_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("out_engage", rclcpp::QoS{10});
  vel_lim_pub_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>("out_velocity_limit", rclcpp::QoS{10});

	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "in_odom", rclcpp::QoS{100}, std::bind(&LoopedMissionPlanner::callbackOdometry, this, std::placeholders::_1));
	odom_msg_ptr_ = nullptr;

  pose_checkpoints_.resize(0);

  loadGoalRosbag();

  auto period_control_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_dt_));
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), period_control_ns, std::bind(&LoopedMissionPlanner::timerCallback, this));

}

LoopedMissionPlanner::~LoopedMissionPlanner() {}

void LoopedMissionPlanner::loadGoalRosbag(){
  {           
    geometry_msgs::msg::Pose checkpoint;             
    checkpoint.position.x = 3773.4169921875;
    checkpoint.position.y = 73696.9765625;
    checkpoint.position.z = 0.0;
    checkpoint.orientation.x = 0.0;
    checkpoint.orientation.y = 0.0;
    checkpoint.orientation.z = -0.9707864937370029;
    checkpoint.orientation.w = 0.2399449594757431;
    pose_checkpoints_.push_back(checkpoint);
  }
  {    
    geometry_msgs::msg::Pose checkpoint;                                    
    checkpoint.position.x = 3697.14990234375;
    checkpoint.position.y = 73725.5078125;
    checkpoint.position.z = 0.0;
    checkpoint.orientation.x = 0.0;
    checkpoint.orientation.y = 0.0;           
    checkpoint.orientation.z = 0.8505706443118113;
    checkpoint.orientation.w = 0.5258607981538366;
    pose_checkpoints_.push_back(checkpoint);
  }
  {    
    geometry_msgs::msg::Pose checkpoint;                                    
    checkpoint.position.x = 3755.300048828125;
    checkpoint.position.y = 73790.9609375;
    checkpoint.position.z = 0.0;
    checkpoint.orientation.x = 0.0;
    checkpoint.orientation.y = 0.0;
    checkpoint.orientation.z = 0.21225468773444692;
    checkpoint.orientation.w = 0.9772143815636118;
    pose_checkpoints_.push_back(checkpoint);
  }
  {    
    geometry_msgs::msg::Pose checkpoint;                                    
    checkpoint.position.x = 3834.58740234375;
    checkpoint.position.y = 73768.1875;
    checkpoint.position.z = 0.0;
    checkpoint.orientation.x = 0.0;
    checkpoint.orientation.y = 0.0;
    checkpoint.orientation.z = -0.48416501714150467;
    checkpoint.orientation.w = 0.8749767060764341;
    pose_checkpoints_.push_back(checkpoint);
  }
  // std::string field = "/planning/mission_planning/goal";
  // const rosbag2_cpp::StorageOptions storage_options(
  //   {rosbag_path_, "sqlite3"});
  // const rosbag2_cpp::ConverterOptions converter_options(
  //   {rmw_get_serialization_format(), rmw_get_serialization_format()});

  // rosbag2_cpp::readers::SequentialReader reader;
  // reader.open(storage_options, converter_options);
  // auto topics = reader.get_all_topics_and_types();
  // std::string topic_type;
  // bool topic_found = false;

  // for (auto t:topics){
  //   if(t.name == field){
  //     topic_type = t.type;
  //     topic_found = true;
  //   }
  // }

  // rosbag2_storage::StorageFilter filter;
  // filter.topics.push_back(field);
  // reader.set_filter(filter);

  // // Read and deserialize "serialized data"
  // if(reader.has_next()){
  //   // Serialized data
  //   auto serialized_message = reader.read_next();
  //   auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
  //   ros_message->time_stamp = 0;
  //   ros_message->message = nullptr;
  //   ros_message->allocator = rcutils_get_default_allocator();
  //   // ros_message->message = msg;
  //   auto type_library = rosbag2_cpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  //   auto type_support = rosbag2_cpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", type_library);

  //   rosbag2_cpp::SerializationFormatConverterFactory factory;
  //   std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
  //   cdr_deserializer_ = factory.load_deserializer("cdr");
  //   cdr_deserializer_->deserialize(serialized_message, type_support, ros_message);
  //   std::cout << 1 << std::endl;
  // }
}

void LoopedMissionPlanner::callbackOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr)
{
  odom_msg_ptr_ = odom_msg_ptr;
}

void LoopedMissionPlanner::timerCallback()
{
	if (odom_msg_ptr_ == nullptr) {return;}

	double distance_to_goal;
	double dx, dy;

  auto current_goal = pose_checkpoints_[num_checkpoints_ % pose_checkpoints_.size()];
  dx = current_goal.position.x - odom_msg_ptr_->pose.pose.position.x;
  dy = current_goal.position.y - odom_msg_ptr_->pose.pose.position.y;
	distance_to_goal = std::sqrt(dx * dx + dy * dy);

  if (distance_to_goal < goal_tolerance_) { // close enough to current coal
		std::cout << "reached goal" << std::endl;

		++num_checkpoints_;

		// publish goal
		geometry_msgs::msg::PoseStamped goal_pose_msg;
		goal_pose_msg.header.stamp = this->now();
		goal_pose_msg.header.frame_id = "map";
    goal_pose_msg.pose = pose_checkpoints_[num_checkpoints_ % pose_checkpoints_.size()];
		goal_pose_pub_->publish(goal_pose_msg);

    std::cout << "next goal: point " << num_checkpoints_ % pose_checkpoints_.size() << std::endl;

    rclcpp::sleep_for(std::chrono::milliseconds(200));

		// publish engage
		autoware_auto_vehicle_msgs::msg::Engage engage_msg;
		engage_msg.engage = true;
		engage_pub_->publish(engage_msg);

		if (num_checkpoints_ % pose_checkpoints_.size() == 0){
			++lap_num_;
			std::cout << "Current lap: " << lap_num_ << std::endl;
		}
	} else { // not reached to the current goal yet
		return;
	}
}
