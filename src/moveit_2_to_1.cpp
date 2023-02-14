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
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/subscription.hpp"

// std::sharedptr<actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>> ros1_client;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class LifecycleNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(const std::string & node_name, const std::string & action_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  { 
    ros2_server_ = rclcpp_action::create_server<FollowJointTrajectory>(this->get_node_base_interface(),
                                                   this->get_node_clock_interface(),
                                                   this->get_node_logging_interface(),
                                                   this->get_node_waitables_interface(),
                                                   action_name,
                                                   std::bind(&LifecycleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                                   std::bind(&LifecycleNode::handle_cancel, this, std::placeholders::_1),
                                                   std::bind(&LifecycleNode::handle_accepted, this, std::placeholders::_1));
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

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request ");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
  }

private:
    std::shared_ptr<rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>> ros2_server_;

};


int main(int argc, char * argv[])
{
  // ROS 2 node, publisher and subscriber
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNode>("moveit_2_to_1", "moveit_action");

  // ROS 1 node and publisher
  ros::init(argc, argv, "moveit_2_to_1");
  ros::NodeHandle n;
  // client = std::make_shared<actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>(n);

  while (rclcpp::ok() && ros::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    ros::spinOnce();
  }

  rclcpp::shutdown();

  return 0;
}
