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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/controller_manager/controller_manager.hpp>

#include "cobot_controller_manager/single_point_trajectory_controller_handle.hpp"

namespace cobot_controller_manager
{

  class CobotControllerManager : public moveit_controller_manager::MoveItControllerManager
  {
  public:
    CobotControllerManager() {}

    ~CobotControllerManager() override = default;

    /*
     * Define controllers, joints and create SinglePointTrajectoryControllerHandle.
     */
    void initialize(const rclcpp::Node::SharedPtr &node)
    {
      node_ = node;
      // TODO @robgineer: do not use hard coded values here
      // identify all parameters from the node
      std::string controller_name = "arm_group_controller";
      std::vector<std::string> joints = {
          "joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      auto handle = std::make_shared<SinglePointTrajectoryControllerHandle>(
          controller_name, "/joint_trajectory_controller/command", node_);
      // set controller and joints
      controllers_[controller_name] = handle;
      controller_joints_[controller_name] = joints;
      // TODO @robgineer: extend this for the grippers
    }

    moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name) override
    {
      return controllers_.at(name);
    }

    /*
     * Provide the list of controllers present.
     *
     * Args:
     *  names: string vector of names to be filled
     */
    void getControllersList(std::vector<std::string> &names) override
    {
      for (const auto &it : controllers_)
      {
        names.push_back(it.first);
      }
    }

    /*
     * Return active controllers. Since there are no inactive controllers, we simply
     * list all controllers.
     *
     * Args:
     *  names: string vector of names to be filled
     */
    void getActiveControllers(std::vector<std::string> &names) override
    {
      getControllersList(names);
    }

    /*
     * By default all controllers are loaded.
     * Calling getControllersList
     *
     * TODO @robgineer: might not be required.
     *
     * Args:
     *  names: string vector of names to be filled
     */
    virtual void getLoadedControllers(std::vector<std::string> &names)
    {
      getControllersList(names);
    }

    /*
     * Return the list of joints for a given controller name
     *
     * Args:
     *  name: name of controller
     *  joints: string vector of joints to be filled
     */
    void getControllerJoints(const std::string &name, std::vector<std::string> &joints) override
    {
      joints = controller_joints_.at(name);
    }

    /*
     * Returns the controller state.
     * We expect the controllers to be always active.
     */
    moveit_controller_manager::MoveItControllerManager::ControllerState
    getControllerState(const std::string & /*name*/) override
    {
      moveit_controller_manager::MoveItControllerManager::ControllerState state;
      state.active_ = true;
      state.default_ = true;
      return state;
    }

    /*
     * Switch controllers.
     * Not required and hence, not implemented.
     */
    bool switchControllers(const std::vector<std::string> & /*activate*/,
                           const std::vector<std::string> & /*deactivate*/) override
    {
      return false;
    }

  protected:
    rclcpp::Node::SharedPtr node_;
    std::map<std::string, std::vector<std::string>> controller_joints_;
    std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controllers_;
  };

} // namespace cobot_controller_manager

// register the plugin
PLUGINLIB_EXPORT_CLASS(cobot_controller_manager::CobotControllerManager,
                       moveit_controller_manager::MoveItControllerManager)
