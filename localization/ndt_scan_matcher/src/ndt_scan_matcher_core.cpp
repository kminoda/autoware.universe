// Copyright 2015-2019 Autoware Foundation
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
// limitations under the License.  std::shared_ptr<NDTBase> * ndt_ptr_ptr_;


#include "ndt_scan_matcher/ndt_scan_matcher_core.hpp"

NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  map_frame_("map")
{
  key_value_stdmap_["state"] = "Initializing";

  bool use_dynamic_map_loading = this->declare_parameter<bool>("use_dynamic_map_loading");
  NDTImplementType ndt_implement_type;
  if (!use_dynamic_map_loading) {
    ndt_implement_type = NDTImplementType::OMP;
    RCLCPP_INFO(get_logger(), "Use static map loading");
  } else {
    ndt_implement_type = NDTImplementType::OMP_MULTI_VOXEL;
    RCLCPP_INFO(get_logger(), "Use dynamic map loading");
  }

  try {
    ndt_ptr_ = get_ndt<PointSource, PointTarget>(ndt_implement_type);
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  NDTBase::BaseParam param;
  param.trans_epsilon = this->declare_parameter<double>("trans_epsilon");
  param.step_size = this->declare_parameter<double>("step_size");
  param.resolution = this->declare_parameter<double>("resolution");
  param.max_iterations = this->declare_parameter<int>("max_iterations");
  param.regularization_scale_factor = this->declare_parameter<float>("regularization_scale_factor");
  ndt_ptr_->setParam(param);

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    param.trans_epsilon, param.step_size, param.resolution, param.max_iterations);

  if (ndt_ptr_->getImplementationType() == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;
    T::OMPParam omp_param;
    int search_method = this->declare_parameter<int>("omp_neighborhood_search_method");
    omp_param.search_method = static_cast<pclomp::NeighborSearchMethod>(search_method);
    omp_param.num_threads = this->declare_parameter<int>("omp_num_threads");
    if (omp_param.num_threads <= 0) {
      RCLCPP_ERROR(get_logger(), "Set valid num_threads (should be num_threads >= 1)");
      return;
    }
    std::dynamic_pointer_cast<T>(ndt_ptr_)->setOMPParam(omp_param);
  } else if (ndt_ptr_->getImplementationType() == NDTImplementType::OMP_MULTI_VOXEL) {
    using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    T::OMPMultiVoxelParam omp_param;
    omp_param.num_threads = this->declare_parameter<int>("omp_num_threads");
    if (omp_param.num_threads <= 0) {
      RCLCPP_ERROR(get_logger(), "Set valid num_threads (should be num_threads >= 1)");
      return;
    }
    std::dynamic_pointer_cast<T>(ndt_ptr_)->setOMPMultiVoxelParam(omp_param);
  }

  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group;
  initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  map_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  tf2_listener_module_ = std::make_shared<Tf2ListenerModule>(this);
  ndt_scan_matching_module_ = std::make_unique<NDTScanMatchingModule>(
    this, &ndt_ptr_mtx_, &ndt_ptr_, tf2_listener_module_, map_frame_,
    main_callback_group, initial_pose_callback_group, &key_value_stdmap_);

  if (!use_dynamic_map_loading) {
    map_module_ = std::make_unique<MapModule>(this, &ndt_ptr_mtx_, &ndt_ptr_, main_callback_group);
    initial_pose_module_ = std::make_unique<InitialPoseModule>(this, &ndt_ptr_mtx_, &ndt_ptr_,
      tf2_listener_module_, map_frame_, main_callback_group, &key_value_stdmap_);
  } else {
    map_update_module_ = std::make_unique<MapUpdateModule>(this, &ndt_ptr_mtx_, &ndt_ptr_,
      tf2_listener_module_, map_frame_, main_callback_group, map_callback_group_, &key_value_stdmap_);
  }

  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_thread_ = std::thread(&NDTScanMatcher::timer_diagnostic, this);
  diagnostic_thread_.detach();
}

void NDTScanMatcher::timer_diagnostic()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }
    // Ignore local optimal solution
    if (
      key_value_stdmap_.count("is_local_optimal_solution_oscillation") &&
      std::stoi(key_value_stdmap_["is_local_optimal_solution_oscillation"])) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_status_msg.message = "local optimal solution oscillation occurred";
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_->publish(diag_msg);

    rate.sleep();
  }
}
