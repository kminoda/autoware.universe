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

#include "ndt_scan_matcher/ndt_scan_matcher_core.hpp"

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/pose_array_interpolator.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <thread>

tier4_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = tier4_debug_msgs::msg::Float32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

tier4_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = tier4_debug_msgs::msg::Int32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

geometry_msgs::msg::TransformStamped identity_transform_stamped(
  const builtin_interfaces::msg::Time & timestamp, const std::string & header_frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = header_frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.rotation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
  transform.transform.translation = tier4_autoware_utils::createTranslation(0.0, 0.0, 0.0);
  return transform;
}

bool validate_local_optimal_solution_oscillation(
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &
    result_pose_matrix_array,
  const float oscillation_threshold, const float inversion_vector_threshold)
{
  bool prev_oscillation = false;
  int oscillation_cnt = 0;
  for (size_t i = 2; i < result_pose_matrix_array.size(); ++i) {
    const Eigen::Vector3f current_pose = result_pose_matrix_array.at(i).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_pose = result_pose_matrix_array.at(i - 1).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_prev_pose = result_pose_matrix_array.at(i - 2).block(0, 3, 3, 1);
    const auto current_vec = (current_pose - prev_pose).normalized();
    const auto prev_vec = (prev_pose - prev_prev_pose).normalized();
    const bool oscillation = prev_vec.dot(current_vec) < inversion_vector_threshold;
    if (prev_oscillation && oscillation) {
      if (oscillation_cnt > oscillation_threshold) {
        return true;
      }
      ++oscillation_cnt;
    } else {
      oscillation_cnt = 0;
    }
    prev_oscillation = oscillation;
  }
  return false;
}

NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  tf2_broadcaster_(*this),
  ndt_implement_type_(NDTImplementType::PCL_GENERIC),
  base_frame_("base_link"),
  ndt_base_frame_("ndt_base_link"),
  map_frame_("map"),
  converged_param_type_(ConvergedParamType::TRANSFORM_PROBABILITY),
  converged_param_transform_probability_(4.5),
  converged_param_nearest_voxel_transformation_likelihood_(2.3),
  initial_estimate_particles_num_(100),
  initial_pose_timeout_sec_(1.0),
  initial_pose_distance_tolerance_m_(10.0),
  inversion_vector_threshold_(-0.9),
  oscillation_threshold_(10),
  regularization_enabled_(declare_parameter("regularization_enabled", false)),
  regularization_scale_factor_(declare_parameter("regularization_scale_factor", 0.01))
{
  key_value_stdmap_["state"] = "Initializing";

  int ndt_implement_type_tmp = this->declare_parameter("ndt_implement_type", 0);
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_implement_type_tmp);

  RCLCPP_INFO(get_logger(), "NDT Implement Type is %d", ndt_implement_type_tmp);
  try {
    ndt_ptr_ = get_ndt<PointSource, PointTarget>(ndt_implement_type_);
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;

    // FIXME(IshitaTakeshi) Not sure if this is safe
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    int search_method = static_cast<int>(omp_params_.search_method);
    search_method = this->declare_parameter("omp_neighborhood_search_method", search_method);
    omp_params_.search_method = static_cast<pclomp::NeighborSearchMethod>(search_method);
    // TODO(Tier IV): check search_method is valid value.
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);

    omp_params_.num_threads = this->declare_parameter("omp_num_threads", omp_params_.num_threads);
    omp_params_.num_threads = std::max(omp_params_.num_threads, 1);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    ndt_ptr_ = ndt_omp_ptr;
  }

  int points_queue_size = this->declare_parameter("input_sensor_points_queue_size", 0);
  points_queue_size = std::max(points_queue_size, 0);
  RCLCPP_INFO(get_logger(), "points_queue_size: %d", points_queue_size);

  base_frame_ = this->declare_parameter("base_frame", base_frame_);
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_.c_str());

  ndt_base_frame_ = this->declare_parameter("ndt_base_frame", ndt_base_frame_);
  RCLCPP_INFO(get_logger(), "ndt_base_frame_id: %s", ndt_base_frame_.c_str());

  double trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  double step_size = ndt_ptr_->getStepSize();
  double resolution = ndt_ptr_->getResolution();
  int max_iterations = ndt_ptr_->getMaximumIterations();
  trans_epsilon = this->declare_parameter("trans_epsilon", trans_epsilon);
  step_size = this->declare_parameter("step_size", step_size);
  resolution = this->declare_parameter("resolution", resolution);
  max_iterations = this->declare_parameter("max_iterations", max_iterations);
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);
  ndt_ptr_->setRegularizationScaleFactor(regularization_scale_factor_);

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    trans_epsilon, step_size, resolution, max_iterations);

  int converged_param_type_tmp = this->declare_parameter("converged_param_type", 0);
  converged_param_type_ = static_cast<ConvergedParamType>(converged_param_type_tmp);
  if (
    ndt_implement_type_ != NDTImplementType::OMP &&
    converged_param_type_ == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    RCLCPP_ERROR(
      get_logger(),
      "ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD is only available when "
      "NDTImplementType::OMP is selected.");
    return;
  }

  converged_param_transform_probability_ = this->declare_parameter(
    "converged_param_transform_probability", converged_param_transform_probability_);
  converged_param_nearest_voxel_transformation_likelihood_ = this->declare_parameter(
    "converged_param_nearest_voxel_transformation_likelihood",
    converged_param_nearest_voxel_transformation_likelihood_);

  initial_estimate_particles_num_ =
    this->declare_parameter("initial_estimate_particles_num", initial_estimate_particles_num_);

  initial_pose_timeout_sec_ =
    this->declare_parameter("initial_pose_timeout_sec", initial_pose_timeout_sec_);

  initial_pose_distance_tolerance_m_ = this->declare_parameter(
    "initial_pose_distance_tolerance_m", initial_pose_distance_tolerance_m_);

  std::vector<double> output_pose_covariance =
    this->declare_parameter<std::vector<double>>("output_pose_covariance");
  for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
    output_pose_covariance_[i] = output_pose_covariance[i];
  }

  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group;
  initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 100,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NDTScanMatcher::callback_map_points, this, std::placeholders::_1), main_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(points_queue_size),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1), main_sub_opt);
  regularization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "regularization_pose_with_covariance", 100,
      std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1));

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  exe_time_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  service_ = this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(
      &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), main_callback_group);

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timer_diagnostic, this);
  diagnostic_thread_.detach();
}

void NDTScanMatcher::timer_diagnostic()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }
    // Ignore local optimal solution
    if (
      key_value_stdmap_.count("is_local_optimal_solution_oscillation") &&
      std::stoi(key_value_stdmap_["is_local_optimal_solution_oscillation"])) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_status_msg.message = "local optimal solution oscillation occurred";
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_->publish(diag_msg);

    rate.sleep();
  }
}

void NDTScanMatcher::service_ndt_align(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  get_transform(map_frame_, req->pose_with_covariance.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto mapTF_initial_pose_msg = transform(req->pose_with_covariance, *TF_pose_to_map_ptr);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    res->success = false;
    res->seq = req->seq;
    RCLCPP_WARN(get_logger(), "No InputTarget");
    return;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    res->success = false;
    res->seq = req->seq;
    RCLCPP_WARN(get_logger(), "No InputSource");
    return;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  res->pose_with_covariance = align_using_monte_carlo(ndt_ptr_, mapTF_initial_pose_msg);
  key_value_stdmap_["state"] = "Sleeping";
  res->success = true;
  res->seq = req->seq;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  // lock mutex for initial pose
  std::lock_guard<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    const builtin_interfaces::msg::Time & t_front =
      initial_pose_msg_ptr_array_.front()->header.stamp;
    const builtin_interfaces::msg::Time & t_msg = initial_pose_msg_ptr->header.stamp;
    if (t_front.sec > t_msg.sec || (t_front.sec == t_msg.sec && t_front.nanosec > t_msg.nanosec)) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
  } else {
    // get TF from pose_frame to map_frame
    auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
    get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    auto mapTF_initial_pose_msg_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    *mapTF_initial_pose_msg_ptr = transform(*initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  regularization_pose_msg_ptr_array_.push_back(pose_conv_msg_ptr);
}

void NDTScanMatcher::callback_map_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  using NDTBase = NormalDistributionsTransformBase<PointSource, PointTarget>;
  std::shared_ptr<NDTBase> new_ndt_ptr = get_ndt<PointSource, PointTarget>(ndt_implement_type_);

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;

    // FIXME(IshitaTakeshi) Not sure if this is safe
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    new_ndt_ptr = ndt_omp_ptr;
  }

  new_ndt_ptr->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr->setStepSize(step_size);
  new_ndt_ptr->setResolution(resolution);
  new_ndt_ptr->setMaximumIterations(max_iterations);
  new_ndt_ptr->setRegularizationScaleFactor(regularization_scale_factor_);

  pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  new_ndt_ptr->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_ptr_mtx_.lock();
  ndt_ptr_ = new_ndt_ptr;
  ndt_ptr_mtx_.unlock();
}

void NDTScanMatcher::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_sensorTF_msg_ptr)
{
  // mutex ndt_ptr_
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  const rclcpp::Time sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;

  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  transform_sensor_measurement(
    sensor_frame, base_frame_, sensor_points_sensorTF_ptr, sensor_points_baselinkTF_ptr);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);

  // calculate initial pose
  std::unique_lock<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  PoseArrayInterpolator interpolator(
    this, sensor_ros_time, initial_pose_msg_ptr_array_, initial_pose_timeout_sec_,
    initial_pose_distance_tolerance_m_);
  if (!interpolator.is_success()) return;
  pop_old_pose(initial_pose_msg_ptr_array_, sensor_ros_time);
  initial_pose_array_lock.unlock();

  // if regularization is enabled and available, set pose to NDT for regularization
  if (regularization_enabled_ && (ndt_implement_type_ == NDTImplementType::OMP))
    add_regularization_pose(sensor_ros_time);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No MAP!");
    return;
  }

  // perform ndt scan matching
  key_value_stdmap_["state"] = "Aligning";
  const NdtResult ndt_result = align(interpolator.get_current_pose().pose.pose);
  key_value_stdmap_["state"] = "Sleeping";

  // perform several validations
  const NdtValidationResult validation_result = validate_ndt_result(ndt_result);

  // publish
  initial_pose_with_covariance_pub_->publish(interpolator.get_current_pose());
  exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, ndt_result.exe_time));
  transform_probability_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
  publish_tf(sensor_ros_time, ndt_result.pose);
  publish_point_cloud(sensor_ros_time, ndt_result.pose, sensor_points_baselinkTF_ptr);
  publish_marker(sensor_ros_time, ndt_result.transformation_array);
  publish_initial_to_result_distances(
    sensor_ros_time, ndt_result.pose, interpolator.get_current_pose(), interpolator.get_old_pose(),
    interpolator.get_new_pose());
  if (validation_result.is_converged)
    publish_pose(sensor_ros_time, ndt_result.pose);

  key_value_stdmap_["transform_probability"] = std::to_string(ndt_result.transform_probability);
  key_value_stdmap_["nearest_voxel_transformation_likelihood"] =
    std::to_string(ndt_result.nearest_voxel_transformation_likelihood);
  key_value_stdmap_["iteration_num"] = std::to_string(ndt_result.iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(validation_result.skipping_publish_num);
  if (validation_result.is_local_optimal_solution_oscillation) {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "1";
  } else {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "0";
  }
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string source_frame, const std::string target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_output_ptr)
{
  auto TF_target_to_source_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  get_transform(target_frame, source_frame, TF_target_to_source_ptr);
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_target_to_source_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
  pcl::transformPointCloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

NdtResult NDTScanMatcher::align(const geometry_msgs::msg::Pose & initial_pose_msg)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  const Eigen::Affine3d initial_pose_affine = from_ros_pose_to_eigen(initial_pose_msg);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);

  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  NdtResult ndt_result;
  ndt_result.pose = tf2::toMsg(result_pose_affine);
  ndt_result.transformation_array = ndt_ptr_->getFinalTransformationArray();
  ndt_result.transform_probability = ndt_ptr_->getTransformationProbability();
  ndt_result.nearest_voxel_transformation_likelihood =
    ndt_ptr_->getNearestVoxelTransformationLikelihood();
  ndt_result.iteration_num = ndt_ptr_->getFinalNumIteration();
  ndt_result.exe_time = exe_time;

  return ndt_result;
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::align_using_monte_carlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses =
    create_random_pose_array(initial_pose_with_cov, initial_estimate_particles_num_);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];
    const NdtResult ndt_result = align(initial_pose);

    Particle particle(
      initial_pose, ndt_result.pose, ndt_result.transform_probability, ndt_result.iteration_num);
    particle_array.push_back(particle);
    const auto marker_array = make_debug_markers(
      this->now(), map_frame_, tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle, i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    publish_point_cloud(
      initial_pose_with_cov.header.stamp, ndt_result.pose, ndt_ptr->getInputSource());
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  // ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);

  return result_pose_with_cov_msg;
}

void NDTScanMatcher::publish_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;
  tf2_broadcaster_.sendTransform(
    tier4_autoware_utils::pose2transform(result_pose_stamped_msg, ndt_base_frame_));
}

void NDTScanMatcher::publish_pose(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = output_pose_covariance_;

  ndt_pose_pub_->publish(result_pose_stamped_msg);
  ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose,
  const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & sensor_points_baselinkTF_ptr)
{
  const Eigen::Affine3d result_pose_affine = from_ros_pose_to_eigen(result_pose);
  const Eigen::Matrix4f result_pose_matrix = result_pose_affine.matrix().cast<float>();

  auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);
}

void NDTScanMatcher::publish_marker(
  const rclcpp::Time & sensor_ros_time,
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &
    result_pose_matrix_array)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose_affine);
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);
}

void NDTScanMatcher::publish_initial_to_result_distances(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  const float initial_to_result_distance =
    norm(initial_pose_cov_msg.pose.pose.position, result_pose_msg.position);
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const float initial_to_result_distance_old =
    norm(initial_pose_old_msg.pose.pose.position, result_pose_msg.position);
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const float initial_to_result_distance_new =
    norm(initial_pose_new_msg.pose.pose.position, result_pose_msg.position);
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_new));
}

bool NDTScanMatcher::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr)
{
  const geometry_msgs::msg::TransformStamped identity =
    identity_transform_stamped(this->now(), target_frame, source_frame);

  if (target_frame == source_frame) {
    *transform_stamped_ptr = identity;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    *transform_stamped_ptr = identity;
    return false;
  }
  return true;
}


NdtValidationResult NDTScanMatcher::validate_ndt_result(const NdtResult & ndt_result)
{
  /*****************************************************************************
  The reason the add 2 to the ndt_ptr_->getMaximumIterations() is that there are bugs in
  implementation of ndt.
  1. gradient descent method ends when the iteration is greater than max_iteration if it does not
  converge (be careful it's 'greater than' instead of 'greater equal than'.)
     https://github.com/tier4/autoware.iv/blob/2323e5baa0b680d43a9219f5fb3b7a11dd9edc82/localization/pose_estimator/ndt_scan_matcher/ndt_omp/include/ndt_omp/ndt_omp_impl.hpp#L212
  2. iterate iteration count when end of gradient descent function.
     https://github.com/tier4/autoware.iv/blob/2323e5baa0b680d43a9219f5fb3b7a11dd9edc82/localization/pose_estimator/ndt_scan_matcher/ndt_omp/include/ndt_omp/ndt_omp_impl.hpp#L217

  These bugs are now resolved in original pcl implementation.
  https://github.com/PointCloudLibrary/pcl/blob/424c1c6a0ca97d94ca63e5daff4b183a4db8aae4/registration/include/pcl/registration/impl/ndt.hpp#L73-L180
  *****************************************************************************/
  bool is_ok_iteration_num =
    validate_num_iteration(ndt_result.iteration_num, ndt_ptr_->getMaximumIterations() + 2);
  bool is_local_optimal_solution_oscillation = false;
  if (!is_ok_iteration_num) {
    is_local_optimal_solution_oscillation = validate_local_optimal_solution_oscillation(
      ndt_result.transformation_array, oscillation_threshold_, inversion_vector_threshold_);
  }
  bool is_ok_converged_param = validate_converged_param(
    ndt_result.transform_probability, ndt_result.nearest_voxel_transformation_likelihood);
  bool is_converged = is_ok_iteration_num && is_ok_converged_param;
  static size_t skipping_publish_num = 0;
  if (is_converged) {
    skipping_publish_num = 0;
  } else {
    ++skipping_publish_num;
    RCLCPP_WARN(get_logger(), "Not Converged");
  }

  NdtValidationResult validation_result;
  validation_result.is_converged = is_converged;
  validation_result.is_local_optimal_solution_oscillation = is_local_optimal_solution_oscillation;
  validation_result.skipping_publish_num = skipping_publish_num;
  return validation_result;
}

bool NDTScanMatcher::validate_num_iteration(const int iter_num, const int max_iter_num)
{
  bool is_ok_iter_num = iter_num < max_iter_num;
  if (!is_ok_iter_num) {
    RCLCPP_WARN(
      get_logger(),
      "The number of iterations has reached its upper limit. The number of iterations: %d, Limit: "
      "%d",
      iter_num, max_iter_num);
  }
  return is_ok_iter_num;
}

bool NDTScanMatcher::validate_score(
  const double score, const double score_threshold, const std::string score_name)
{
  bool is_ok_score = score > score_threshold;
  if (!is_ok_score) {
    RCLCPP_WARN(
      get_logger(), "%s is below the threshold. Score: %lf, Threshold: %lf", score_name.c_str(),
      score, score_threshold);
  }
  return is_ok_score;
}

bool NDTScanMatcher::validate_converged_param(
  const double & transform_probability, const double & nearest_voxel_transformation_likelihood)
{
  bool is_ok_converged_param = false;
  if (converged_param_type_ == ConvergedParamType::TRANSFORM_PROBABILITY) {
    is_ok_converged_param = validate_score(
      transform_probability, converged_param_transform_probability_, "Transform Probability");
  } else if (converged_param_type_ == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    is_ok_converged_param = validate_score(
      nearest_voxel_transformation_likelihood,
      converged_param_nearest_voxel_transformation_likelihood_,
      "Nearest Voxel Transformation Likelihood");
  } else {
    is_ok_converged_param = false;
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Unknown converged param type.");
  }
  return is_ok_converged_param;
}

std::optional<Eigen::Matrix4f> NDTScanMatcher::interpolate_regularization_pose(
  const rclcpp::Time & sensor_ros_time)
{
  if (regularization_pose_msg_ptr_array_.empty()) {
    return std::nullopt;
  }

  // synchronization
  PoseArrayInterpolator interpolator(this, sensor_ros_time, regularization_pose_msg_ptr_array_);

  pop_old_pose(regularization_pose_msg_ptr_array_, sensor_ros_time);

  // if the interpolate_pose fails, 0.0 is stored in the stamp
  if (rclcpp::Time(interpolator.get_current_pose().header.stamp).seconds() == 0.0) {
    return std::nullopt;
  }

  Eigen::Affine3d regularization_pose_affine;
  tf2::fromMsg(interpolator.get_current_pose().pose.pose, regularization_pose_affine);
  return regularization_pose_affine.matrix().cast<float>();
}

void NDTScanMatcher::add_regularization_pose(const rclcpp::Time & sensor_ros_time)
{
  ndt_ptr_->unsetRegularizationPose();
  std::optional<Eigen::Matrix4f> pose_opt = interpolate_regularization_pose(sensor_ros_time);
  if (pose_opt.has_value()) {
    ndt_ptr_->setRegularizationPose(pose_opt.value());
    RCLCPP_DEBUG_STREAM(get_logger(), "Regularization pose is set to NDT");
  }
}
