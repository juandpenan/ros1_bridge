// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pub;

class LifecycleNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    pub->on_activate();

    RCLCPP_INFO(get_logger(), "on_activate()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    pub->on_deactivate();

    RCLCPP_INFO(get_logger(), "on_deactivate()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    pub.reset();

    RCLCPP_INFO(get_logger(), "on cleanup()");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    pub.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  friend void OdomCallback(boost::shared_ptr<nav_msgs::Odometry> ros1_msg);
};

void OdomCallback(boost::shared_ptr<nav_msgs::Odometry> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<nav_msgs::msg::Odometry>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());
  ros2_msg->child_frame_id = ros1_msg->child_frame_id;
  ros2_msg->pose.pose.position.x = ros1_msg->pose.pose.position.x;
  ros2_msg->pose.pose.position.y = ros1_msg->pose.pose.position.y;
  ros2_msg->pose.pose.position.z = ros1_msg->pose.pose.position.z;
  ros2_msg->pose.pose.orientation.x = ros1_msg->pose.pose.orientation.x;
  ros2_msg->pose.pose.orientation.y = ros1_msg->pose.pose.orientation.y;
  ros2_msg->pose.pose.orientation.z = ros1_msg->pose.pose.orientation.z;
  ros2_msg->pose.pose.orientation.w = ros1_msg->pose.pose.orientation.w;
  
  for (long unsigned int i = 0; i < ros1_msg->pose.covariance.size(); i++) {
    ros2_msg->pose.covariance[i] = ros1_msg->pose.covariance[i];
  }
  
    for (long unsigned int i = 0; i < ros1_msg->twist.covariance.size(); i++) {
    ros2_msg->twist.covariance[i] = ros1_msg->twist.covariance[i];
  }

  ros2_msg->twist.twist.linear.x  = ros1_msg->twist.twist.linear.x;
  ros2_msg->twist.twist.linear.y  = ros1_msg->twist.twist.linear.y;
  ros2_msg->twist.twist.linear.z  = ros1_msg->twist.twist.linear.z;
  ros2_msg->twist.twist.angular.x  = ros1_msg->twist.twist.angular.x;
  ros2_msg->twist.twist.angular.y  = ros1_msg->twist.twist.angular.y;
  ros2_msg->twist.twist.angular.z  = ros1_msg->twist.twist.angular.z;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNode>("odom_1_to_2");

  // ROS 1 node and subscriber
  ros::init(argc, argv, "odom_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/mobile_base_controller/odom", 100, OdomCallback);

  while (rclcpp::ok() && ros::ok()) {
    ros::spinOnce();
    rclcpp::spin_some(node->get_node_base_interface());
  }


  rclcpp::shutdown();

  return 0;
}
