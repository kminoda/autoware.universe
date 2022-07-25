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

#ifndef MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_

#include "autoware_map_msgs/srv/load_pcd_partially.hpp"
#include "autoware_map_msgs/srv/load_pcd_partially_for_publish.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_map_msgs/msg/pcd_map_array.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
  bool use_downsample_;
  bool enable_whole_load_;
  bool enable_partial_load_;
  float leaf_size_;

  std::string map_frame_;
  std::string viewer_frame_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_whole_pointcloud_map_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_partial_pointcloud_map_;
  rclcpp::Publisher<autoware_map_msgs::msg::PCDMapArray>::SharedPtr pub_partial_pointcloud_maps_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr pub_debug_loading_time_ms_;
  rclcpp::Service<autoware_map_msgs::srv::LoadPCDPartially>::SharedPtr load_pcd_partially_service_;
  rclcpp::Service<autoware_map_msgs::srv::LoadPCDPartiallyForPublish>::SharedPtr
    load_pcd_partially_for_publish_service_;

  sensor_msgs::msg::PointCloud2 loadPCDFiles(const std::vector<std::string> & pcd_paths);
  struct PCDFileMetadata
  {
    pcl::PointXYZ min;
    pcl::PointXYZ max;
  };
  std::map<std::string, PCDFileMetadata> current_pcd_file_metadata_dict_;
  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
  void generatePCDMetadata(
    const std::vector<std::string> & pcd_paths);

  // sensor_msgs::msg::PointCloud2 loadPCDPartially(
  //   const geometry_msgs::msg::Point position, const float radius,
  //   std::vector<PCDFileMetadata> pcd_file_metadata_array) const;

  autoware_map_msgs::msg::PCDMapArray loadPCDPartially(
    const geometry_msgs::msg::Point position, const float radius);

  bool updateLoadedMaps(
    const geometry_msgs::msg::Point position, const float radius);
  bool loadPCDPartiallyForPublishServiceCallback(
    autoware_map_msgs::srv::LoadPCDPartiallyForPublish::Request::SharedPtr req,
    autoware_map_msgs::srv::LoadPCDPartiallyForPublish::Response::SharedPtr res);
  bool loadPCDPartiallyServiceCallback(
    autoware_map_msgs::srv::LoadPCDPartially::Request::SharedPtr req,
    autoware_map_msgs::srv::LoadPCDPartially::Response::SharedPtr res);
  void generateTF(sensor_msgs::msg::PointCloud2 & pcd_msg);
};

#endif  // MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
