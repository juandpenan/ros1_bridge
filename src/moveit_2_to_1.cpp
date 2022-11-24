// Copyright 2022 Intelligent Robotics Lab
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

#include <iostream>
#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/subscription.hpp"

class LifecycleNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  { }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on cleanup()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;

};

void poseCallback(const geometry_msgs::msg::Pose::SharedPtr ros2_msg)
{
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = ros2_msg->position.x;
  goal_pose.pose.position.y = ros2_msg->position.y;
  goal_pose.pose.position.z = ros2_msg->pose.position.z;
  goal_pose.pose.orientation.x = ros2_msg->orientation.x;
  goal_pose.pose.orientation.x = ros2_msg->orientation.y;
  goal_pose.pose.orientation.x = ros2_msg->orientation.z;
  goal_pose.pose.orientation.x = ros2_msg->orientation.w;

  // Select group
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(5.0);

  bool success = bool(group_arm_torso.plan(my_plan));

  // Execute the plan
  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
  if (!bool(e))
    throw std::runtime_error("Error executing plan");
}

int main(int argc, char * argv[])
{
  // ROS 2 node, publisher and subscriber
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNode>("moveit_2_to_1");
  auto sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "/tiago_arm/goal", 100, poseCallback);

  // ROS 1 node and publisher
  ros::init(argc, argv, "moveit_2_to_1");
  ros::NodeHandle n;

  while (rclcpp::ok() && ros::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    ros::spinOnce();
  }

  rclcpp::shutdown();

  return 0;
}

