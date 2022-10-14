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
// limitations under the License.

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_

#include "ndt_scan_matcher/tf2_listener_module.hpp"
#include "ndt_scan_matcher/ndt_scan_matching_module.hpp"
#include "ndt_scan_matcher/map_module.hpp"
#include "ndt_scan_matcher/initial_pose_module.hpp"
#include "ndt_scan_matcher/map_update_module.hpp"

#include <ndt/omp.hpp>
#include <ndt/omp_multi_voxel.hpp>
// #include <ndt/pcl_generic.hpp>
// #include <ndt/pcl_modified.hpp>
#include <ndt/utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable>

class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NDTBase = NormalDistributionsTransformBase<PointSource, PointTarget>;

public:
  NDTScanMatcher();

private:
  void timer_diagnostic();

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> backup_ndt_ptr_;
  std::string map_frame_;
  std::mutex ndt_ptr_mtx_;

  std::thread diagnostic_thread_;
  std::map<std::string, std::string> key_value_stdmap_;

  rclcpp::CallbackGroup::SharedPtr map_callback_group_;

  std::shared_ptr<Tf2ListenerModule> tf2_listener_module_;
  std::unique_ptr<NDTScanMatchingModule> ndt_scan_matching_module_;
  std::unique_ptr<MapModule> map_module_;
  std::unique_ptr<InitialPoseModule> initial_pose_module_;
  std::unique_ptr<MapUpdateModule> map_update_module_;
};

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
