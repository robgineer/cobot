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

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "cobot_msgs/srv/cobot_api_srv.hpp"

namespace cobot_rviz_plugins
{ /*
   * Implementation of a control panel for custom commands
   * for the cobot_trajectory_controller and the cobot hw interface.
   * 
   * Custom commands are send via the CobotApiService.
   */
  class CobotControlPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit CobotControlPanel(QWidget *parent = 0);
    ~CobotControlPanel() override;

  private Q_SLOTS:
    void sendCommands();

  private:
    QCheckBox *acknowledge_error_checkbox_;
    QCheckBox *request_abort_checkbox_;
    QCheckBox *single_point_checkbox_;
    QCheckBox *full_trajectory_checkbox_;
    QDoubleSpinBox *resampling_delta_spinbox_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<cobot_msgs::srv::CobotApiSrv>::SharedPtr cobot_api_service_;
  };
} // namespace cobot_rviz_plugins
