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
#include <QVBoxLayout>
#include <QPushButton>
#include <QButtonGroup>
#include <QLabel>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "cobot_rviz_plugins/cobot_control_panel.hpp"

namespace cobot_rviz_plugins
{
  CobotControlPanel::CobotControlPanel(QWidget *parent) : Panel(parent)
  {
    node_ = rclcpp::Node::make_shared("cobot_control_panel");

    auto layout = new QVBoxLayout;

    /**** cobot_trajectory_controller settings ****/
    QButtonGroup *trajectory_controller_group = new QButtonGroup(this);
    single_point_checkbox_ = new QCheckBox("single_point", this);
    full_trajectory_checkbox_ = new QCheckBox("full_trajectory", this);
    trajectory_controller_group->addButton(single_point_checkbox_);
    trajectory_controller_group->addButton(full_trajectory_checkbox_);
    // single_point and full_trajectory are mutually exclusive
    trajectory_controller_group->setExclusive(true);
    // set as default
    single_point_checkbox_->setChecked(true);
    layout->addWidget(new QLabel("Execution Mode:", this));
    layout->addWidget(single_point_checkbox_);
    layout->addWidget(full_trajectory_checkbox_);
    // resampling_delta for "full_trajectory" mode
    resampling_delta_spinbox_ = new QDoubleSpinBox(this);
    resampling_delta_spinbox_->setRange(0.0, 10.0);
    resampling_delta_spinbox_->setSingleStep(0.01);
    // set to 3deg as default
    resampling_delta_spinbox_->setValue(0.53);
    layout->addWidget(new QLabel("resampling_delta:", this));
    layout->addWidget(resampling_delta_spinbox_);

    /**** acknowledge error and request abort (Cobot API specific) ****/
    //QButtonGroup *cobot_api_group = new QButtonGroup(this);
    acknowledge_error_checkbox_ = new QCheckBox("Acknowledge Error", this);
    //cobot_api_group->addButton(acknowledge_error_checkbox_);
    // we restrict to sending one command at the same time
    //cobot_api_group->setExclusive(true);
    layout->addWidget(acknowledge_error_checkbox_);

    QPushButton *send_button = new QPushButton("Send", this);
    layout->addWidget(send_button);
    setLayout(layout);
    connect(send_button, &QPushButton::clicked, this, &CobotControlPanel::sendCommands);
    // connect to cobot api service
    cobot_api_service_ = node_->create_client<cobot_msgs::srv::CobotApiSrv>("cobot_api_service");
  }

  CobotControlPanel::~CobotControlPanel() = default;

  void CobotControlPanel::sendCommands()
  {
    auto request = std::make_shared<cobot_msgs::srv::CobotApiSrv::Request>();
    // since the checkboxes for the execution mode are mutually exclusive,
    // we need to check only one
    request->request_full_trajectory_mode = full_trajectory_checkbox_->isChecked();

    request->resampling_delta = resampling_delta_spinbox_->value();
    request->acknowledge_error = acknowledge_error_checkbox_->isChecked();

    if (!cobot_api_service_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
      return;
    }

    auto future = cobot_api_service_->async_send_request(request);

    RCLCPP_WARN(node_->get_logger(), "Custom Cobot controls sent.");
  }

} // namespace cobot_rviz_plugins

PLUGINLIB_EXPORT_CLASS(cobot_rviz_plugins::CobotControlPanel, rviz_common::Panel)