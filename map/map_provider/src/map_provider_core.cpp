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

#include "map_provider/map_provider_core.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

bool needs_update_map(
  const geometry_msgs::msg::Vector3 p1, const geometry_msgs::msg::Point p2, const double e)
{
  return std::pow(e, 2.0) <=
         std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0) + std::pow(p1.z - p2.z, 2.0);
}

MapProvider::MapProvider() : Node("map_provider"), tf2_listener_(tf2_buffer_)
{
  pointcloud_map_radius_ = this->declare_parameter("pointcloud_map_radius", 50.0);
  update_threshold_distance_ = this->declare_parameter("update_threshold_distance", 100.0);

  maps_pub_ = this->create_publisher<autoware_map_msgs::msg::PCDMapArray>(
    "output/pointcloud_map", rclcpp::QoS(1).transient_local());

  pcd_loader_service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pcd_loader_client_ = this->create_client<autoware_map_msgs::srv::LoadPCDPartiallyForPublish>(
    "pcd_loader_service", rmw_qos_profile_services_default, pcd_loader_service_group_);
  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_DEBUG(get_logger(), "Waiting for pcd_loader_service...");
  }

  // update_map_timer_ = create_wall_timer(
  //   std::chrono::microseconds(1000), std::bind(&MapProvider::updateMapTimerCallback, this));
  double map_provider_dt_ = 1.0;
  auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(map_provider_dt_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&MapProvider::timerCallback, this));

  pcd_loader_res_ = nullptr;
}

void MapProvider::timerCallback()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer_.lookupTransform(
      "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(), "Could NOT transform pose %s frame to %s frame : %s", "map", "base_link",
      ex.what());
    return;
  }

  if (pcd_loader_res_) {
    // check distance
    if (!needs_update_map(
          transform_stamped.transform.translation, pcd_loader_res_->position,
          update_threshold_distance_)) {
      // no need to update
      return;
    } else {
      std::cout << "need an update??????????" << std::endl;
    }
  } else {
    std::cout << "pcd_loader_res_ = nullptr" << std::endl;
  }

  // update map
  auto request = std::make_shared<autoware_map_msgs::srv::LoadPCDPartiallyForPublish::Request>();
  request->position.x = transform_stamped.transform.translation.x;
  request->position.y = transform_stamped.transform.translation.y;
  request->position.z = transform_stamped.transform.translation.z;
  request->radius = pointcloud_map_radius_;
  auto result{pcd_loader_client_->async_send_request(
    request,
    [this](const rclcpp::Client<autoware_map_msgs::srv::LoadPCDPartiallyForPublish>::SharedFuture response) {
      (void)response;
      std::lock_guard<std::mutex> lock{mutex_};
      value_ready_ = true;
      condition_.notify_all();
    })};
  std::unique_lock<std::mutex> lock{mutex_};
  condition_.wait(lock, [this]() { return value_ready_; });
  // TODO: This may be wrong. Maybe must return when failed to call pcd_loader_ server? (koji
  // minoda)

  pcd_loader_res_ = result.get();

  // if ((int(pcd_loader_res_->maps.pcd_maps.size()) == 0) & (int(pcd_loader_res_->maps.removing_cloud_ids.size()) == 0)) {
  //   // RCLCPP_ERROR(
  //   //   get_logger(), "No Map Update! pos=%lf, %lf %lf", request->position.x, request->position.y,
  //   //   request->position.z);
  //   return;
  // }

  // maps_pub_->publish(pcd_loader_res_->maps);
}
