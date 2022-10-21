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
#include "tf2_msgs/TFMessage.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>::SharedPtr pub;

class LifecycleNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    pub = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(100).keep_all().transient_local());
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

  friend void TFCallback(const tf2_msgs::TFMessage::ConstPtr & ros1_msg);
};

void TFCallback(const tf2_msgs::TFMessage::ConstPtr & ros1_msg)
{
  // std::cout << "I heard: [" << ros1_msg->data << "]" << std::endl;

  auto ros2_msg = std::make_unique<tf2_msgs::msg::TFMessage>();

  for (const auto & transform : ros1_msg->transforms) {
    geometry_msgs::msg::TransformStamped transform2;
    transform2.header.frame_id = transform.header.frame_id;
    transform2.header.stamp = rclcpp::Time(transform.header.stamp.toNSec());
    transform2.child_frame_id = transform.child_frame_id;
    transform2.transform.translation.x = transform.transform.translation.x;
    transform2.transform.translation.y = transform.transform.translation.y;
    transform2.transform.translation.z = transform.transform.translation.z;
    transform2.transform.rotation.x = transform.transform.rotation.x;
    transform2.transform.rotation.y = transform.transform.rotation.y;
    transform2.transform.rotation.z = transform.transform.rotation.z;
    transform2.transform.rotation.w = transform.transform.rotation.w;
    ros2_msg->transforms.push_back(transform2);
  }
  // ros2_msg->transforms = ros1_msg->transforms;
  //std::cout << "Passing along: [" << ros2_msg->data << "]" << std::endl;
  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNode>("tf_static_1_to_2");
  
  // ROS 1 node and subscriber
  ros::init(argc, argv, "tf_static_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/tf_static", 100, TFCallback);

  while (rclcpp::ok() && ros::ok()) {
    ros::spinOnce();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
