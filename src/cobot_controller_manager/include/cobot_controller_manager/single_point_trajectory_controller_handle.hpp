/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Robert Harbach
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/controller_manager/controller_manager.hpp>

namespace cobot_controller_manager
{

  /*
   * A TrajectoryControllerHandle that sends out only the last trajectory point.
   */
  class SinglePointTrajectoryControllerHandle : public moveit_controller_manager::MoveItControllerHandle
  {
  public:
    SinglePointTrajectoryControllerHandle(const std::string &name,
                                          const std::string &topic_name,
                                          const rclcpp::Node::SharedPtr &node)
        : moveit_controller_manager::MoveItControllerHandle(name),
          topic_name_(topic_name),
          node_(node)
    {
      // create action client
      auto action_name = "/arm_group_controller/follow_joint_trajectory"; // TODO @rharbach: figure out a way to avoid hard coded values
      controller_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, action_name);
      if (!controller_action_client_->wait_for_action_server(std::chrono::seconds(5)))
      {
        RCLCPP_ERROR(node_->get_logger(), "Could not find action server within 5 seconds");
        return;
      }
    }

    /*
     * Fill the trajectory and send it to the action sever.
     * Since our hardware interface accepts only one trajectory point,
     * we send out the last trajectory point.
     *
     * Args:
     *  trajectory: the trajectory to be send to the action server
     *
     * Returns: true if action server received trajectory, false otherwise.
     */
    bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override
    {
      if (trajectory.joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(node_->get_logger(), "Received an empty trajectory.");
        return false;
      }

      // prepare trajectory. We are sending out ONE single point of the entire trajectory
      auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
      goal_msg.trajectory.header.stamp = rclcpp::Time(0);
      goal_msg.trajectory.joint_names = trajectory.joint_trajectory.joint_names;
      // use last point only
      auto last_trajectory_point = trajectory.joint_trajectory.points.back();
      // this module -> ROS2 controller -> cobot_hardware
      // since the ROS2 controller interpolates between trajectory points and hence, generates another trajectory,
      // we need to tell it that there is no need for trajectory generation by setting the time from start to 0.
      last_trajectory_point.time_from_start = rclcpp::Duration(0, 0);
      goal_msg.trajectory.points.push_back(last_trajectory_point);

      // send goal to ROS2 controller
      auto current_goal_future = controller_action_client_->async_send_goal(goal_msg);

      if (!current_goal_future.get())
      {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        return false;
      }
      return true;
    }

    /*
     * Cancel the current trajectory execution.
     * Our trajectory's last point is forwarded via ROS2 control directly to the hw interface.
     * => Implementation not required.
     */
    bool cancelExecution() override
    {
      RCLCPP_ERROR(node_->get_logger(), "cancelExecution() is not implemented");
      return true;
    }

    /*
     * Wait for the execution to complete. Implementation not required.
     */
    bool waitForExecution(const rclcpp::Duration &timeout) override
    {
      RCLCPP_ERROR(node_->get_logger(), "waitForExecution() is not implemented");
      return true;
    }

    /*
     * Wait for the execution to complete. Implementation not required.
     */
    moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
    {
      return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    }

  private:
    std::string topic_name_;
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr controller_action_client_;
  };

} // namespace cobot_controller_manager