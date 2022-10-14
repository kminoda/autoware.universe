// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_

#include "differential_map_loader_module.hpp"
#include "partial_map_loader_module.hpp"
#include "pointcloud_map_loader_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>
#include <set>
#include <string>
#include <vector>

using PointTye = pcl::PointXYZI;
constexpr size_t N_SAMPLES = 20;

std::vector<size_t> UniformRandom(const size_t max_exclusive, const size_t n)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<size_t> distribution(0, max_exclusive - 1);

  std::vector<size_t> v(n);
  for (size_t i = 0; i < n; i++) {
    v[i] = distribution(generator);
  }
  return v;
}

class PointCloudMapLoaderNode : public rclcpp::Node
{
public:
  explicit PointCloudMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  // ros param
  std::map<std::string, PCDFileMetadata> pcd_metadata_dict_;

  std::unique_ptr<PointcloudMapLoaderModule> pcd_map_loader_;
  std::unique_ptr<PointcloudMapLoaderModule> downsampled_pcd_map_loader_;
  std::unique_ptr<PartialMapLoaderModule> partial_map_loader_;
  std::unique_ptr<DifferentialMapLoaderModule> differential_map_loader_;

  std::vector<std::string> getPcdPaths(
    const std::vector<std::string> & pcd_paths_or_directory) const;
  std::map<std::string, PCDFileMetadata> generatePCDMetadata(
    const std::vector<std::string> & pcd_paths) const;
};

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
