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

#ifndef NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
#define NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_

#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/tf2_listener_module.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>

#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <ndt/base.hpp>
#include <ndt/omp_multi_voxel.hpp>
#include <ndt/utils.hpp>
#include <fmt/format.h>
#include <pcl_conversions/pcl_conversions.h>

class MapUpdateModule {
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NDTBase = NormalDistributionsTransformBase<PointSource, PointTarget>;

public:
  MapUpdateModule(
    rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
    std::shared_ptr<NDTBase> * ndt_ptr,
    std::shared_ptr<Tf2ListenerModule> tf2_listener_module,
    std::string map_frame,
    rclcpp::CallbackGroup::SharedPtr main_callback_group,
    rclcpp::CallbackGroup::SharedPtr map_callback_group,
    std::map<std::string, std::string> * key_value_stdmap_ptr);

private:
  void service_ndt_align(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);
  void callback_ekf_odom(nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr);
  void map_update_timer_callback();

  void update_ndt_with_new_map(const std::vector<autoware_map_msgs::msg::PointCloudMapCellWithID> & maps_to_add,
    const std::vector<std::string> & map_ids_to_remove);
  void update_map(const geometry_msgs::msg::Point & position);
  bool should_update_map(const geometry_msgs::msg::Point & position);
  std::vector<std::string> get_current_map_ids(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr);
  void publish_partial_pcd_map();
  geometry_msgs::msg::PoseWithCovarianceStamped align_using_monte_carlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const std::shared_ptr<const pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;

  rclcpp::Service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr service_;
  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr pcd_loader_client_;
  rclcpp::TimerBase::SharedPtr map_update_timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  std::shared_ptr<NDTBase> * ndt_ptr_ptr_;
  std::shared_ptr<NDTBase> backup_ndt_ptr_;
  std::mutex * ndt_ptr_mutex_;
  std::string map_frame_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module_;
  std::map<std::string, std::string> * key_value_stdmap_ptr_;

  int initial_estimate_particles_num_;
  std::mutex pcd_loader_client_mutex_;
  std::mutex initial_pose_pcd_loader_client_mutex_;
  std::condition_variable condition_;
  bool value_ready_ = false;
  geometry_msgs::msg::Point::SharedPtr last_update_position_ptr_;
  double dml_update_map_distance_;
  double dml_loading_radius_;
  geometry_msgs::msg::Point::SharedPtr current_position_ptr_;

};

#endif  // NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_