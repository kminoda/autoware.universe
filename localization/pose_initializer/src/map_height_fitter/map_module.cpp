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

#include "map_module.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <memory>

MapModule::MapModule(rclcpp::Node * node, bool partial_map_load_enabled)
: logger_(node->get_logger()),
  partial_map_load_enabled_(partial_map_load_enabled)
{
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  using std::placeholders::_1;

  if (!partial_map_load_enabled_) {
    sub_map_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_map", durable_qos, std::bind(&MapModule::on_map, this, _1));
  } else {
    callback_group_services_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cli_get_partial_pcd_ = node->create_client<autoware_map_msgs::srv::GetPartialPointCloudMap>(
      "client_partial_map_load", rmw_qos_profile_default, callback_group_services_);
    while (!cli_get_partial_pcd_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
      RCLCPP_INFO(logger_, "Waiting for pcd loader service...");
    }
  }
}

void MapModule::on_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *map_cloud_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapModule::get_map_ptr() const
{
  return map_cloud_;
}

std::string MapModule::get_map_frame() const
{
  return map_frame_;
}

void MapModule::get_partial_point_cloud_map(geometry_msgs::msg::Point & point)
{
  if (!cli_get_partial_pcd_) {
    throw std::runtime_error{"Partial map loading is not enabled"};
  }
  const auto req = std::make_shared<autoware_map_msgs::srv::GetPartialPointCloudMap::Request>();
  req->area.center = point;
  req->area.radius = 50;

  RCLCPP_INFO(logger_, "Send request to map_loader");
  auto res {
    cli_get_partial_pcd_->async_send_request(
      req,
      [](rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedFuture){})
  };

  std::future_status status = res.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(logger_, "waiting response");
    if (!rclcpp::ok()) {return;}
    status = res.wait_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(logger_, "Loaded partial pcd map from map_loader (grid size: %d)", int(res.get()->new_pointcloud_with_ids.size()));

  sensor_msgs::msg::PointCloud2 pcd_msg;
  for (const auto & pcd_with_id : res.get()->new_pointcloud_with_ids) {
    if (pcd_msg.width == 0) {
      pcd_msg = pcd_with_id.pointcloud;
    } else {
      pcd_msg.width += pcd_with_id.pointcloud.width;
      pcd_msg.row_step += pcd_with_id.pointcloud.row_step;
      pcd_msg.data.insert(pcd_msg.data.end(),
        pcd_with_id.pointcloud.data.begin(), pcd_with_id.pointcloud.data.end());
    }
  }

  map_frame_ = res.get()->frame_id;
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pcd_msg, *map_cloud_);
}
