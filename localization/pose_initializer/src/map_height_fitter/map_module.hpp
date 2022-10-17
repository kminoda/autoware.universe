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

#ifndef MAP_HEIGHT_FITTER__MAP_MODULE_HPP_
#define MAP_HEIGHT_FITTER__MAP_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>

class MapModule
{
public:
  explicit MapModule(rclcpp::Node * node, bool use_partial_map_load);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_map_ptr() const;
  std::string get_map_frame() const;
  void get_partial_point_cloud_map(geometry_msgs::msg::Point & point);

private:
  void on_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  
  rclcpp::Logger logger_;
  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedPtr cli_get_partial_pcd_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  bool partial_map_load_enabled_;
};

#endif  // MAP_HEIGHT_FITTER__MAP_MODULE_HPP_
