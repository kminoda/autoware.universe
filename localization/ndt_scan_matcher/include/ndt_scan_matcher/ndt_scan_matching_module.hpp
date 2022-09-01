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

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHING_MODULE_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHING_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt/omp.hpp>
#include <ndt/pcl_generic.hpp>
#include <ndt/pcl_modified.hpp>

struct AlignResults{

};

struct InitialAlignResults{

};

class NdtScanMatchingModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NDTBase = NormalDistributionsTransformBase<PointSource, PointTarget>;

public:
  NdtScanMatchingModule();
  AlignResults setInputSensorPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void setInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  InitialAlignResults alignUsingMonteCarlo(
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

  void setNdtPtr(std::shared_ptr<NDTBase> & input);
  void copyNdtPtr(std::shared_ptr<NDTBase> & output);

private:
  bool validateTimeStampDifference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec);
  bool validatePositionDifference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_);

  std::shared_ptr<NDTBase> ndt_ptr_;

  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    initial_pose_msg_ptr_array_;
  std::mutex ndt_map_mtx_;
  std::mutex initial_pose_array_mtx_;
}


#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHING_MODULE_HPP_
