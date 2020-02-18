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
#include "sensor_msgs/PointCloud2.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;


void TFCallback(boost::shared_ptr<sensor_msgs::PointCloud2> ros1_msg)
{
  if (pub->get_subscription_count() == 0)
    return;

  auto ros2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

  ros2_msg->header.frame_id = ros1_msg->header.frame_id;
  ros2_msg->header.stamp = rclcpp::Time(ros1_msg->header.stamp.toNSec());

  ros2_msg->height = ros1_msg->height;
  ros2_msg->width = ros1_msg->width;

  for (auto & f1 : ros1_msg->fields) {
    sensor_msgs::msg::PointField f2;
    f2.name = f1.name;
    f2.offset = f1.offset;
    f2.datatype = f1.datatype;
    f2.count = f1.count;
    ros2_msg->fields.push_back(f2);
  }

  ros2_msg->is_bigendian = ros1_msg->is_bigendian;
  ros2_msg->point_step = ros1_msg->point_step;
  ros2_msg->row_step = ros1_msg->row_step;
  ros2_msg->data = std::move(ros1_msg->data);
  ros2_msg->is_dense = ros1_msg->is_dense;

  pub->publish(std::move(ros2_msg));
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pc2_1_to_2");
  pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/points", 100);

  // ROS 1 node and subscriber
  ros::init(argc, argv, "pc2_1_to_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 100, TFCallback);

  ros::spin();

  rclcpp::shutdown();

  return 0;
}
