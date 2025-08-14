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
#include <vector>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "cobot_msgs/srv/cobot_api_srv.hpp"

namespace cobot_trajectory_controller
{
  /*
   * Implementation of a JointTrajectoryController tailored for the Cobot.
   * Unlike the standard implementation this controller does not use real time
   * joint commands (command interfaces) for the control of the hardware.
   * Instead, it accepts a trajectory and passes either its last point or
   * the entire trajectory to the hardware interface using a real time buffer (singleton).
   * Rationale: the Cobot to be controlled does not accept single commands for trajectory
   * execution but either the final trajectory point or a list of trajectory points.
   *
   * Sending one point or the entire trajectory is configured using the parameter "execution_mode",
   * which enables on demand changes of the trajectory execution. This parameter can be updated
   * in the cobot_control_panel in rviz.
   *
   * Note: the usage of a singleton buffer might violate some ROS2 control principles but
   * it is an acceptable tradeoff. Without it we would either need to send complex
   * data structures through the as double declared CommandInterfaces (via reinterpret_cast)
   * or implement a custom CommandInterfaces that can be handled by the ROS2 control manager.
   *
   */
  class CobotTrajectoryController : public controller_interface::ControllerInterface
  {
  public:
    CobotTrajectoryController();

    /*
     * Expose the command interfaces to the controller manager
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    /*
     * Expose the state interfaces to the controller manager
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    /*
     * Configure ROS2 environment: get parameter values, initialise action server, etc.
     */
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    /*
     * Identifies the index of the custom commands that are used for sending custom cobot control commands.
     */
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    /*
     * Cleanup
     */
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    /*
     * Communication with the hardware interface. Fills a realtime buffer with a trajectory for the hardware interface.
     */
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    // not required pure virtual function
    controller_interface::CallbackReturn on_init() override;

  private:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using ServerGoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
    /*
     * Called once new trajectory is sent from MoveIt to the running action server.
     * This function fills the internal_trajectory_buffer_.
     * The reason behind using an internal buffer is the asynchronous execution of the callback.
     * It is executed once a new trajectory is available and hence runs in parallel to the update
     * function of this controller. In order to avoid race conditions, we write the trajectory into
     * an internal buffer and read it within the update function in real time.
     */
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    /*
     * Implementation is not required. We do not react on cancellations (we use the request_abort that
     * is set in the cobot_control_panel in rviz) and hence we simply return with
     * rclcpp_action::CancelResponse:ACCEPT.
     */
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<ServerGoalHandle> goal_handle);
    /*
     * Implementation is not required. We do not do anything special within the scope of a trajectory execution and hence
     * we simply set the goal handle to "succeeded".
     */
    void accepted_callback(
        const std::shared_ptr<ServerGoalHandle> goal_handle);
    /*
     * Spatial trajectory resampling for the reduction of points.
     *
     * In case full trajectory forwarding is active (execution_mode == "full_trajectory"), we forward the entire
     * trajectory to the hardware interface.
     * Since the Cobot does not accept timing based commands but a path and since dense trajectory points
     * imply jerky movements, we resample the trajectory based on a minimum distance between two subsequent
     * trajectory points. This distance represents the maximum velocity of the Cobot and can be adjusted
     * using the parameter: "resampling_delta".
     * Note: a higher resampling_delta implies less trajectory points. For moving around obstacles, we would
     * need at least 3 points.
     *
     * The resampling is rather simple but effective for the Cobot. We iterate through the trajectory points
     * starting from the last point and use all points that have a minimum angular delta of "resampling_delta_".
     * Since we have multiple joints, we orient ourselves on the fastest moving joint.
     * Note: joint_0, the prismatic joint, is ignored (we only use its final trajectory point for control).
     *
     * Example with resampling_delta = 0.5
     *
     * Trajectory point:     0     1     2     3     4     5     6     7     8
     * --------------------------------------------------------------------------
     * Joint_1 pos (rad):   0.0   0.2   0.4   0.6   0.8   1.0   1.2   1.4   1.6
     * Selected:             -     -     #     -     -     #     -     -     #
     * Joint_2 pos (rad):   0.0   0.2   0.4   0.6   0.8   1.0   1.0   1.0   1.0
     * Selected:             -     -     -     -     -     -     -     -     #
     *
     *
     * # = selected point
     * - = skipped point (delta < resampling_delta)
     *
     * Resampled trajectory point:     0     1     2
     * -----------------------------------------------
     * Resampled joint_1 pos (rad):   0.4   1.0   1.6
     * Resampled joint_1 pos (rad):   0.4   1.0   1.0
     *
     * Note: in practice all joints move with the same velocity
     * (we have defined the same limits for each joint in the configurations)
     *
     * Args:
     *  trajectory: the trajectory to be resampled
     * Returns:
     *  resampled trajectory
     */
    trajectory_msgs::msg::JointTrajectory resample_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory);
    /*
     * Initiate the service (cobot_api_service_) between this controller
     * and the rviz panel that sends out custom commands.
     * Since communication between the rviz panel and this controller is rare, we are using a service.
     *
     * The service offers the following custom commands / settings:
     * 1. set execution_mode (single_point or full_trajectory)
     * 2. set the resampling_delta
     * 3. acknowledge error in hardware
     * 4. request abort in hardware
     */
    void setup_service_for_rviz_panel();

    // The minumum distance in rad between two trajectory points (used in resample_trajectory())
    float resampling_delta_ = 0.53;
    // Logger of this node (to avoid calling get_node()->get_logger() frequently)
    std::unique_ptr<rclcpp::Logger> local_logger_;
    // Define if only the last point or the full trajectory will be send to the hardware interface
    std::string execution_mode_ = "single_point";
    // Action server for the trajectory. Communicates with MoveIt2 and executes the callbacks declared above.
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    // Names of joints to be claimed in this controller
    std::vector<std::string> joint_names_;
    using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
    // Internal buffer. Filled once a new trajectory has been sent to the action server
    realtime_tools::RealtimeBuffer<std::optional<JointTrajectory>> internal_trajectory_buffer_;
    // External buffer (singleton). Filled in case a new trajectory is available. Consumed by the hardware interface.
    std::shared_ptr<realtime_tools::RealtimeBuffer<std::optional<JointTrajectory>>> external_trajectory_buffer_;
    // The instance of the communication service between the rviz panel and this controller
    using CobotApiSrv = cobot_msgs::srv::CobotApiSrv;
    rclcpp::Service<CobotApiSrv>::SharedPtr cobot_api_service_;
    // The indexes of the custom API commands that are forwarded to the hardware interface
    int acknowledge_error_command_index_;
    int request_abort_command_index_;
    // The values for the custom API commands
    double acknowledge_error_; // hw interface only accepts double values
    double request_abort_;
  };

} // namespace cobot_trajectory_controller
