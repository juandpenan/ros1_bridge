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
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
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

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowJointTrajectoryRos1 = control_msgs::FollowJointTrajectoryAction;
using ROS2Goal = typename FollowJointTrajectory::Goal;
using ROS1Goal = typename actionlib::ActionServer<FollowJointTrajectoryRos1>::Goal;
using ROS1Client = typename actionlib::ActionClient<FollowJointTrajectoryRos1>;
using ROS1ClientGoalHandle = typename actionlib::ActionClient<FollowJointTrajectoryRos1>::GoalHandle;
using ROS2Result = typename FollowJointTrajectory::Result;
using ROS1Result = typename actionlib::ActionServer<FollowJointTrajectoryRos1>::Result;
using ROS2Feedback = typename FollowJointTrajectory::Feedback;
using ROS1Feedback = typename actionlib::ActionServer<FollowJointTrajectoryRos1>::Feedback;

class LifecycleNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(ros::NodeHandle ros1_node, const std::string & node_name, const std::string & action_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    node_handle_ = ros1_node;
    ros1_client_ = std::make_shared<ROS1Client>(node_handle_, action_name);

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
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> ros2_goal_handle)
  {
    (void)ros2_goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  template <typename T1, typename T2>
    static void copy_point(const T1 &pt1, T2 &pt2)
  {
    pt2.positions = pt1.positions;
    pt2.velocities = pt1.velocities;
    pt2.accelerations = pt1.accelerations;
  }

  template <typename T1, typename T2>
    static void copy_tolerance(const T1 &tolerance1, T2 &tolerance2)
  {
    tolerance2.name = tolerance1.name;
    tolerance2.position = tolerance1.position;
    tolerance2.velocity = tolerance1.velocity;
    tolerance2.acceleration = tolerance1.acceleration;
  }

  static void
  copy_duration_2_to_1(const builtin_interfaces::msg::Duration &duration2,
                     ros::Duration &duration1)
  {
    duration1.sec = duration2.sec;
    duration1.nsec = duration2.nanosec;
  }


  template <typename T1, typename T2>
    static void copy_tolerances(const T1 &t1, T2 &t2)
  {
    const size_t num = t1.size();
    t2.resize(num);
    for (size_t i = 0; i < num; ++i)
    {
      copy_tolerance(t1[i], t2[i]);
    }
  }

  void translate_goal_2_to_1(
    const ROS2Goal &goal2, ROS1Goal &goal1)
  {
    goal1.trajectory.joint_names = goal2.trajectory.joint_names;
    const size_t num = goal2.trajectory.points.size();
    goal1.trajectory.points.resize(num);

    for (size_t i = 0; i < num; ++i)
    {
      copy_point(goal2.trajectory.points[i], goal1.trajectory.points[i]);
      copy_duration_2_to_1(goal2.trajectory.points[i].time_from_start,
                          goal1.trajectory.points[i].time_from_start);
    }

    copy_tolerances(goal2.path_tolerance, goal1.path_tolerance);
    copy_tolerances(goal2.goal_tolerance, goal1.goal_tolerance);

    copy_duration_2_to_1(goal2.goal_time_tolerance, goal1.goal_time_tolerance);
  }

  void translate_result_1_to_2(
      ROS2Result &result2, const ROS1Result &result1)
  {
    result2.error_code = result1.error_code;
    result2.error_string = result1.error_string;
  }

  void translate_feedback_1_to_2(
      ROS2Feedback &feedback2, const ROS1Feedback &feedback1)
  {
    feedback2.joint_names = feedback1.joint_names;
    copy_point(feedback1.desired, feedback2.desired);
    copy_point(feedback1.actual, feedback2.actual);
    copy_point(feedback1.error, feedback2.error);
  }

  void handle_accepted(const
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> ros2_goal_handle)
  {
    (void)ros2_goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted");

    auto goal2 = ros2_goal_handle->get_goal();

    ROS1Goal goal1;
    translate_goal_2_to_1(*goal2, goal1);

    std::condition_variable cond_result;
    std::atomic<bool> result_ready(false);

    auto ros1_goal_handle = ros1_client_->sendGoal(
          goal1,
          [this, &ros2_goal_handle, &result_ready, &cond_result](ROS1ClientGoalHandle ros1_goal_handle) mutable // transition_cb
          {
            ROS_INFO("Goal [%s]", ros1_goal_handle.getCommState().toString().c_str());

            if (ros1_goal_handle.getCommState() == actionlib::CommState::RECALLING)
            {
              //cancelled before being processed
              auto result2 = std::make_shared<ROS2Result>();
              ros2_goal_handle->canceled(result2);

              return;
            }
            else if (ros1_goal_handle.getCommState() == actionlib::CommState::ACTIVE)
            {
              std::lock_guard<std::mutex> lock(mutex_);
              ros1_goal_handle_ = std::make_shared<ROS1ClientGoalHandle>(ros1_goal_handle);
            } else if (ros1_goal_handle.getCommState() == actionlib::CommState::DONE)
            {
              auto result2 = std::make_shared<ROS2Result>();
              auto result1 = ros1_goal_handle.getResult();
              translate_result_1_to_2(*result2, *result1);
              ROS_INFO("Goal [%s]", ros1_goal_handle.getTerminalState().toString().c_str());

              if (ros1_goal_handle.getTerminalState() == actionlib::TerminalState::SUCCEEDED)
              {
                ros2_goal_handle->succeed(result2);
              }
              else
              {
                ros2_goal_handle->abort(result2);
              }
              result_ready = true;
              cond_result.notify_one();
              return;
            }
          },
          [this, &ros2_goal_handle](ROS1ClientGoalHandle ros1_goal_handle, auto feedback1) mutable //feedback_cb
          {
            (void)ros1_goal_handle;
            auto feedback2 = std::make_shared<ROS2Feedback>();
            translate_feedback_1_to_2(*feedback2, *feedback1);
            ros2_goal_handle->publish_feedback(feedback2);
          });
  }

private:
    std::shared_ptr<rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>> ros2_server_;
    std::shared_ptr<ROS1Client> ros1_client_;
    std::shared_ptr<ROS1ClientGoalHandle> ros1_goal_handle_;
    ros::NodeHandle node_handle_;
    std::mutex mutex_;
};


int main(int argc, char * argv[])
{
  // ROS 1 node and publisher
  ros::init(argc, argv, "moveit_2_to_1");
  ros::NodeHandle n;

  // ROS 2 node, publisher and subscriber
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleNode>(n, "moveit_2_to_1", "/gripper_controller/follow_joint_trajectory");

  while (rclcpp::ok() && ros::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    ros::spinOnce();
  }

  rclcpp::shutdown();

  return 0;
}
