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

#include <pluginlib/class_list_macros.hpp>

#include "cobot_hardware/shared_trajectory_buffer.hpp"
#include "cobot_trajectory_controller/cobot_trajectory_controller.hpp"

namespace cobot_trajectory_controller
{

  CobotTrajectoryController::CobotTrajectoryController() {}

  controller_interface::InterfaceConfiguration CobotTrajectoryController::command_interface_configuration() const
  {
    // Claim all joints for this controller; so MoveIt and ROS2 are satisfied.
    // The actual command is send in form of the full trajectory to the hardware interface.
    // We use the singleton buffer (external_trajectory_buffer_) for this. Refer update function.
    controller_interface::InterfaceConfiguration command_interface_configuration;
    command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : joint_names_)
    {
      command_interface_configuration.names.push_back(joint + "/position");
    }
    return command_interface_configuration;
  }

  controller_interface::InterfaceConfiguration CobotTrajectoryController::state_interface_configuration() const
  {
    // Register this controller as the updater for the state of each joint.
    // This enables the ROS2 control manager to publish the state of the Cobot by reading
    // the state interfaces provided in the hardware interface.
    controller_interface::InterfaceConfiguration state_interface_configuration;
    state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : joint_names_)
    {
      state_interface_configuration.names.push_back(joint + "/position");
      state_interface_configuration.names.push_back(joint + "/velocity");
    }
    return state_interface_configuration;
  }

  controller_interface::CallbackReturn CobotTrajectoryController::on_configure(const rclcpp_lifecycle::State &)
  {
    local_logger_ = std::make_unique<rclcpp::Logger>(get_node()->get_logger());
    // create action server and register callbacks
    trajectory_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        get_node()->get_node_base_interface(),
        get_node()->get_node_clock_interface(),
        get_node()->get_node_logging_interface(),
        get_node()->get_node_waitables_interface(),
        std::string(get_node()->get_name()) + "/follow_joint_trajectory", // the trajectory we are looking for
        std::bind(&CobotTrajectoryController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CobotTrajectoryController::cancel_callback, this, std::placeholders::_1),
        std::bind(&CobotTrajectoryController::accepted_callback, this, std::placeholders::_1));

    // get the joint names from the URDF
    get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    if (!get_node()->get_parameter("joints", joint_names_))
    {
      RCLCPP_ERROR(*local_logger_, "Parameter 'joints' not set");
      return controller_interface::CallbackReturn::ERROR;
    }
    // initialise the singleton buffer
    // TODO @rharbach: think of error handling
    external_trajectory_buffer_ = SharedTrajectoryBuffer::getTrajectoryBuffer();
    // declare the execution mode
    get_node()->declare_parameter<std::string>("execution_mode", execution_mode_);
    // declare the minimum resampling delta
    get_node()->declare_parameter<float>("resampling_delta", resampling_delta_);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CobotTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // reset internal and external buffer
    internal_trajectory_buffer_.writeFromNonRT(std::nullopt);
    external_trajectory_buffer_->writeFromNonRT(std::nullopt);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type CobotTrajectoryController::update(const rclcpp::Time &, const rclcpp::Duration &)
  {
    // get current trajectory
    auto full_trajectory = internal_trajectory_buffer_.readFromRT();
    // proceed only in case new trajectory is available
    if (full_trajectory && full_trajectory->has_value())
    {
      // handle execution modes
      get_node()->get_parameter("execution_mode", execution_mode_);
      if (execution_mode_ == "full_trajectory")
      {
        // we need to reduce the number of points of the trajectory so the
        // hardware interface can handle the list execution
        auto resampled_trajectory = resample_trajectory(full_trajectory->value());
        external_trajectory_buffer_->writeFromNonRT(resampled_trajectory);
      }
      else if (execution_mode_ == "single_point")
      {
        trajectory_msgs::msg::JointTrajectory single_point_trajectory;
        single_point_trajectory.header.stamp = full_trajectory->value().header.stamp;
        single_point_trajectory.joint_names = full_trajectory->value().joint_names;
        // use last point only
        auto last_trajectory_point = full_trajectory->value().points.back();
        single_point_trajectory.points.push_back(last_trajectory_point);
        external_trajectory_buffer_->writeFromNonRT(single_point_trajectory);
      }
      else
      {
        RCLCPP_ERROR(*local_logger_, "Invalid execution mode. Set execution_mode parameter to single_point or full_trajectory and reset controller.");
        return controller_interface::return_type::ERROR;
      }

      // reset the buffer
      internal_trajectory_buffer_.writeFromNonRT(std::nullopt);
    }

    return controller_interface::return_type::OK;
  }

  rclcpp_action::GoalResponse CobotTrajectoryController::goal_callback(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    internal_trajectory_buffer_.writeFromNonRT(goal->trajectory);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  trajectory_msgs::msg::JointTrajectory CobotTrajectoryController::resample_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory)
  {
    // get resampling delta from parameter
    get_node()->get_parameter("resampling_delta", resampling_delta_);
    // start with the last point
    auto reference_point_index = trajectory.points.size() - 1;
    trajectory_msgs::msg::JointTrajectory resampled_trajectory;
    // always add last point of trajectory (this is the one we absolutely need)
    resampled_trajectory.points.push_back(trajectory.points[reference_point_index]);
    // iterate backwards through the trajectory points starting from the last point
    for (int current_point_index = trajectory.points.size() - 1; current_point_index >= 0; current_point_index--)
    {
      const auto &reference_point = trajectory.points[reference_point_index]; // initially the last point in the traj.
      const auto &next_point = trajectory.points[current_point_index];

      // create potential new point
      trajectory_msgs::msg::JointTrajectoryPoint new_point;
      new_point.positions.resize(trajectory.joint_names.size()); // to access the joint positions via index
      // do not add new point until the delta is big enough
      auto add_new_point = false;
      // we do not use time_from_start but it doesn't hurt to set it right
      new_point.time_from_start = next_point.time_from_start;
      // iterate trough the joints and consider revolute joints only
      // => we do not need resampling on the prismatic joint (for that one we send out the last position only).
      for (size_t joint_index = 1; joint_index < reference_point.positions.size() - 1; joint_index++)
      {
        const auto delta = next_point.positions[joint_index] - reference_point.positions[joint_index];
        if (std::abs(delta) >= resampling_delta_)
        {
          new_point.positions[joint_index] = next_point.positions[joint_index];
          reference_point_index = current_point_index;
          add_new_point = true;
        }
        else
        {
          // nothing has changed for this joint => use reference point.
          new_point.positions[joint_index] = reference_point.positions[joint_index];
        }
      }
      if (add_new_point)
      {
        // at least one joint had delta >= resampling_delta_
        // => add this point to the resampled trajectory
        resampled_trajectory.points.push_back(new_point);
      }
    }
    // since we started with the last point and iterated backwards we, need to reverse the resampled_trajectory
    std::reverse(resampled_trajectory.points.begin(), resampled_trajectory.points.end());

    return resampled_trajectory;
  }

  controller_interface::CallbackReturn CobotTrajectoryController::on_init()
  {
    // not required and hence not implemented
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn CobotTrajectoryController::on_activate(const rclcpp_lifecycle::State &)
  {
    // not required and hence not implemented
    return controller_interface::CallbackReturn::SUCCESS;
  }
  rclcpp_action::CancelResponse CobotTrajectoryController::cancel_callback(const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    // not required and hence not implemented
    goal_handle->canceled(std::make_shared<FollowJointTrajectory::Result>());
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void CobotTrajectoryController::accepted_callback(const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    // not required and hence not implemented
    goal_handle->succeed(std::make_shared<FollowJointTrajectory::Result>());
  }

} // namespace cobot_trajectory_controller

PLUGINLIB_EXPORT_CLASS(cobot_trajectory_controller::CobotTrajectoryController, controller_interface::ControllerInterface)
