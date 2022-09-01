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

#include "localization_managing_module.hpp"

#include <memory>

LocalizationManagingModule::LocalizationManagingModule(rclcpp::Node * node) : logger_(node->get_logger())
{
  client_ekf_shutdown_ = node->create_client<RequestShutdownNode>("ekf_shutdown_node");
  client_ekf_start_ = node->create_client<RequestStartNode>("ekf_start_node");
  client_ndt_shutdown_ = node->create_client<RequestShutdownNode>("ndt_shutdown_node");
  client_ndt_start_ = node->create_client<RequestStartNode>("ndt_start_node");
}

void LocalizationManagingModule::shutdown() const
{
  const auto req = std::make_shared<RequestShutdownNode::Request>();
  const auto res_ekf = client_ekf_shutdown_->async_send_request(req).get();
  const auto res_ndt = client_ndt_shutdown_->async_send_request(req).get();
}

void LocalizationManagingModule::start() const
{
  const auto req = std::make_shared<RequestStartNode::Request>();
  const auto res_ekf = client_ekf_start_->async_send_request(req).get();
  const auto res_ndt = client_ndt_start_->async_send_request(req).get();
}
