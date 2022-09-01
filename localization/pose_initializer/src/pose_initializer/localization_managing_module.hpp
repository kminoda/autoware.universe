// Copyright 2022 The Autoware Contributors
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

#ifndef POSE_INITIALIZER__LOCALIZATION_MANAGING_MODULE_HPP_
#define POSE_INITIALIZER__LOCALIZATION_MANAGING_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_localization_msgs/srv/shutdown_node.hpp>
#include <tier4_localization_msgs/srv/start_node.hpp>

class LocalizationManagingModule
{
private:
  using RequestShutdownNode = tier4_localization_msgs::srv::ShutdownNode;
  using RequestStartNode = tier4_localization_msgs::srv::StartNode;

public:
  explicit LocalizationManagingModule(rclcpp::Node * node);
  void shutdown() const;
  void start() const;

private:
  rclcpp::Logger logger_;
  rclcpp::Client<RequestShutdownNode>::SharedPtr client_ekf_shutdown_;
  rclcpp::Client<RequestShutdownNode>::SharedPtr client_ndt_shutdown_;
  rclcpp::Client<RequestStartNode>::SharedPtr client_ekf_start_;
  rclcpp::Client<RequestStartNode>::SharedPtr client_ndt_start_;
};

#endif  // POSE_INITIALIZER__LOCALIZATION_MANAGING_MODULE_HPP_
