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
#include "sensor_msgs/Imu.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub;

class LifecycleNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    pub = this->create_publisher<sensor_msgs::msg::Imu>("/base_imu", 100);
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

  friend void imuCallback(boost::shared_ptr<sensor_msgs::Imu> ros1_msg);
};

void imuCallback(boost::shared_ptr<sensor_msgs::Imu> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<sensor_msgs::msg::Imu>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());
  
  ros2_msg->orientation.x = ros1_msg->orientation.x;
  ros2_msg->orientation.y = ros1_msg->orientation.y;
  ros2_msg->orientation.z = ros1_msg->orientation.z;
  ros2_msg->orientation.w = ros1_msg->orientation.w;
  std::array<double, 9> o_data, a_data, l_data;
  int i = 0, j = 0, k = 0;
  
  for (auto & data : ros1_msg->orientation_covariance)
  {
    o_data[i] = data;
    i++;
  }
  ros2_msg->orientation_covariance = o_data;

  for (auto & data : ros1_msg->angular_velocity_covariance)
  {
    a_data[j] = data;
    j++;
  }
  ros2_msg->angular_velocity_covariance = a_data;

  for (auto & data : ros1_msg->linear_acceleration_covariance)
  {
    l_data[k] = data;
    k++;
  }
  ros2_msg->linear_acceleration_covariance = l_data;

  ros2_msg->angular_velocity.x = ros1_msg->angular_velocity.x;
  ros2_msg->angular_velocity.y = ros1_msg->angular_velocity.y;
  ros2_msg->angular_velocity.z = ros1_msg->angular_velocity.z;
  ros2_msg->linear_acceleration.x = ros1_msg->linear_acceleration.x;
  ros2_msg->linear_acceleration.y = ros1_msg->linear_acceleration.y;
  ros2_msg->linear_acceleration.z = ros1_msg->linear_acceleration.z;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNode>("imu_1_to_2");

  // ROS 1 node and subscriber
  ros::init(argc, argv, "imu_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/base_imu", 100, imuCallback);

  while (rclcpp::ok() && ros::ok()) {
    ros::spinOnce();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
