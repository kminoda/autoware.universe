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

#ifndef MAP_PROVIDER__MAP_PROVIDER_CORE_HPP_
#define MAP_PROVIDER__MAP_PROVIDER_CORE_HPP_

#include "autoware_map_msgs/srv/load_pcd_partially.hpp"
#include "autoware_map_msgs/msg/pcd_map_array.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/optional.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

class MapProvider : public rclcpp::Node
{
public:
  MapProvider();
  ~MapProvider() = default;

private:
  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_;

  // ros param
  float pointcloud_map_radius_;
  float update_threshold_distance_;

  autoware_map_msgs::srv::LoadPCDPartially::Response::SharedPtr pcd_loader_res_;

  rclcpp::Publisher<autoware_map_msgs::msg::PCDMapArray>::SharedPtr maps_pub_;
  rclcpp::Client<autoware_map_msgs::srv::LoadPCDPartially>::SharedPtr pcd_loader_client_;
  // rclcpp::TimerBase::SharedPtr update_map_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr pcd_loader_service_group_;

  std::mutex mutex_;
  std::condition_variable condition_;
  bool value_ready_ = false;

  void timerCallback();
};

#endif  // MAP_PROVIDER__MAP_PROVIDER_CORE_HPP_
