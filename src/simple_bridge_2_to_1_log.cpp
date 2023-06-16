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
#include "rcl_interfaces/msg/Log.h"

#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"


rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr pub;

void topic_callback(const rcl_interfaces::Log::ConstPtr & ros1_msg)
{
  auto ros2_msg = std::make_unique<rcl_interfaces::msg::Log>();
  
  ros2_msg->DEBUG = ros1_msg->DEBUG;
  ros2_msg->INFO = ros1_msg->INFO;
  ros2_msg->WARN = ros1_msg->WARN;
  ros2_msg->ERROR = ros1_msg->ERROR;
  ros2_msg->FATAL = ros1_msg->FATAL;

  ros2_msg->level = ros1_msg->level;
  ros2_msg->name = ros1_msg->name;
  ros2_msg->msg = ros1_msg->msg;
  ros2_msg->file = ros1_msg->file;
  ros2_msg->function = ros1_msg->function;
  ros2_msg->line = ros1_msg->line;  

  ros2_msg->stamp.sec = ros1_msg->stamp.sec;
  ros2_msg->stamp.nanosec = ros1_msg->stamp.nanosec;


  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{

    static const rmw_qos_profile_t rmw_qos_profile_default =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    50,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      rmw_qos_profile_default.history,
      rmw_qos_profile_default.depth
    ),
    rmw_qos_profile_default);

  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bridge_talker_log");
  node->declare_parameter("topic_name", "topic");
  std::string topic_name =  node->get_parameter("topic_name").get_parameter_value().get<std::string>();
  pub = node->create_publisher<nav_msgs::msg::Odometry>(topic_name, qos);

  // ROS 1 node and subscriber
  ros::init(argc, argv, "bridge_listener_log");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_name, 50, topic_callback);

  ros::spin();

  return 0;
}
