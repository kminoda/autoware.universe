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

#define FMT_HEADER_ONLY

#include "ndt_scan_matcher/particle.hpp"

#include <ndt/omp.hpp>
#include <ndt/omp_multi_voxel.hpp>
#include <ndt/pcl_generic.hpp>
#include <ndt/pcl_modified.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/msg/pcd_map_array.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "autoware_map_msgs/srv/load_pcd_maps_general.hpp"
#include "autoware_map_msgs/msg/area_info.hpp"
#include "autoware_map_msgs/msg/pcd_map_with_id.hpp"

#include <fmt/format.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable>

enum class NDTImplementType { PCL_GENERIC = 0, PCL_MODIFIED = 1, OMP = 2, OMP_MULTI_VOXEL = 3 };
enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

template <typename PointSource, typename PointTarget>
std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> getNDT(
  const NDTImplementType & ndt_mode)
{
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr;
  if (ndt_mode == NDTImplementType::PCL_GENERIC) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::PCL_MODIFIED) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::OMP) {
    ndt_ptr.reset(new NormalDistributionsTransformOMP<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::OMP_MULTI_VOXEL) {
    ndt_ptr.reset(new NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>);
    return ndt_ptr;
  }
  const std::string s = fmt::format("Unknown NDT type {}", static_cast<int>(ndt_mode));
  throw std::runtime_error(s);
}

template <typename PointSource, typename PointTarget>
void copyNDT(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & input_ndt_ptr,
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & output_ndt_ptr,
  const NDTImplementType & ndt_mode)
{
  // std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr;
  // if (ndt_mode == NDTImplementType::PCL_GENERIC) {
  //   using T = NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>;
  //   std::shared_ptr<T> input_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(input_ndt_ptr);
  //   std::shared_ptr<T> output_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(output_ndt_ptr);
  //   *output_ndt_tmp_ptr = *input_ndt_tmp_ptr;
  // }
  // else if (ndt_mode == NDTImplementType::PCL_MODIFIED) {
  //   using T = NormalDistributionsTransformPCLModified<PointSource, PointTarget>;
  //   std::shared_ptr<T> input_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(input_ndt_ptr);
  //   std::shared_ptr<T> output_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(output_ndt_ptr);
  //   *output_ndt_tmp_ptr = *input_ndt_tmp_ptr;
  // }
  // else if (ndt_mode == NDTImplementType::OMP) {
  //   using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;
  //   std::shared_ptr<T> input_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(input_ndt_ptr);
  //   std::shared_ptr<T> output_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(output_ndt_ptr);
  //   *output_ndt_tmp_ptr = *input_ndt_tmp_ptr;
  // } else
  if (ndt_mode == NDTImplementType::OMP_MULTI_VOXEL) {
    using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    std::shared_ptr<T> input_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(input_ndt_ptr);
    std::shared_ptr<T> output_ndt_tmp_ptr = std::dynamic_pointer_cast<T>(output_ndt_ptr);
    output_ndt_tmp_ptr->copyFrom(*input_ndt_tmp_ptr);
    // output_ndt_tmp_ptr->setNumThreads(input_ndt_tmp_ptr->getNumThreads());
    // output_ndt_tmp_ptr->setTransformationEpsilon(input_ndt_tmp_ptr->getTransformationEpsilon());
    // output_ndt_tmp_ptr->setStepSize(input_ndt_tmp_ptr->getStepSize());
    // output_ndt_tmp_ptr->setResolution(input_ndt_tmp_ptr->getResolution());
    // output_ndt_tmp_ptr->setMaximumIterations(input_ndt_tmp_ptr->getMaximumIterations());
    *output_ndt_tmp_ptr = *input_ndt_tmp_ptr;
  } else {
    const std::string s = fmt::format("Unknown NDT type {}", static_cast<int>(ndt_mode));
    throw std::runtime_error(s);
  }
}

class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;

  // TODO(Tier IV): move file
  struct OMPParams
  {
    OMPParams() : search_method(pclomp::NeighborSearchMethod::KDTREE), num_threads(1) {}
    pclomp::NeighborSearchMethod search_method;
    int num_threads;
  };

public:
  NDTScanMatcher();

private:
  void serviceNDTAlign(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  // void callbackMapPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  // void callbackMapPoints(autoware_map_msgs::msg::PCDMapArray::ConstSharedPtr pcd_map_array_msg_ptr);
  std::vector<std::string> getMapIDsToRemove(const std::vector<std::string> & map_ids_ndt_will_possess);
  void updateNDT(const std::vector<autoware_map_msgs::msg::PCDMapWithID> & maps_to_add,
    const std::vector<std::string> & map_ids_to_remove);
  void callbackSensorPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callbackInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  void callbackRegularizationPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  void publishPartialPCDMap();
  geometry_msgs::msg::PoseWithCovarianceStamped alignUsingMonteCarlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

  void updateTransforms();

  void publishTF(
    const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr);

  bool validateTimeStampDifference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec);
  bool validatePositionDifference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_);

  std::optional<Eigen::Matrix4f> interpolateRegularizationPose(
    const rclcpp::Time & sensor_ros_time);

  void timerDiagnostic();
  bool hasCompatibleMap(const geometry_msgs::msg::Point & initial_point);

  // void publishPartialPCDMap(
  //   const autoware_map_msgs::msg::PCDMapArray::SharedPtr pcd_map_array_msg_ptr);

  void mapUpdateTimerCallback();
  void updateMap(const geometry_msgs::msg::Point & position);
  bool shouldUpdateMap(const geometry_msgs::msg::Point & position);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::PCDMapArray>::SharedPtr map_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    regularization_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_with_covariance_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr transform_probability_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Int32Stamped>::SharedPtr iteration_num_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_old_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_new_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ndt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr koji_map_pub_;

  rclcpp::Service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr service_;
  rclcpp::Client<autoware_map_msgs::srv::LoadPCDMapsGeneral>::SharedPtr pcd_loader_client_;
  rclcpp::TimerBase::SharedPtr map_update_timer_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  NDTImplementType ndt_implement_type_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> backup_ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  std::string base_frame_;
  std::string ndt_base_frame_;
  std::string map_frame_;

  ConvergedParamType converged_param_type_;
  double converged_param_transform_probability_;
  double converged_param_nearest_voxel_transformation_likelihood_;

  int initial_estimate_particles_num_;
  double initial_pose_timeout_sec_;
  double initial_pose_distance_tolerance_m_;
  float inversion_vector_threshold_;
  float oscillation_threshold_;
  std::array<double, 36> output_pose_covariance_;
  double initial_ndt_align_timeout_sec_;

  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    initial_pose_msg_ptr_array_;
  std::mutex ndt_map_mtx_;
  std::mutex initial_pose_array_mtx_;

  OMPParams omp_params_;

  double min_x_{0}, min_y_{0}, max_x_{0}, max_y_{0};

  std::thread diagnostic_thread_;
  std::map<std::string, std::string> key_value_stdmap_;

  // variables for regularization
  const bool regularization_enabled_;
  const float regularization_scale_factor_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    regularization_pose_msg_ptr_array_;

  std::mutex pcd_loader_client_mutex_;
  std::condition_variable condition_;
  bool value_ready_ = false;
  geometry_msgs::msg::Point::SharedPtr last_update_position_ptr_;
  double update_threshold_distance_;
  double loading_radius_;
  
};

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
