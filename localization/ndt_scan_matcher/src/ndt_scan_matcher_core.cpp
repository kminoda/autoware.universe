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

// template <typename T>
// void swap(T& a, T& a)
// {
//     T temp = a;
//     a = b;
//     b = temp;
// }

tier4_debug_msgs::msg::Float32Stamped makeFloat32Stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = tier4_debug_msgs::msg::Float32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

tier4_debug_msgs::msg::Int32Stamped makeInt32Stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = tier4_debug_msgs::msg::Int32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

geometry_msgs::msg::TransformStamped identityTransformStamped(
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

double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(
    std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0) + std::pow(p1.z - p2.z, 2.0));
}

bool isLocalOptimalSolutionOscillation(
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

template <typename T, typename U>
double norm_xy(const T p1, const U p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
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
  initial_ndt_align_timeout_sec_(3.0),
  regularization_enabled_(declare_parameter("regularization_enabled", false)),
  regularization_scale_factor_(declare_parameter("regularization_scale_factor", 0.01)),
  dml_update_map_distance_(declare_parameter("dml_update_map_distance", 10)),
  dml_loading_radius_(declare_parameter("dml_loading_radius", 100))
{
  key_value_stdmap_["state"] = "Initializing";

  int ndt_implement_type_tmp = this->declare_parameter("ndt_implement_type", 0);
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_implement_type_tmp);

  RCLCPP_INFO(get_logger(), "NDT Implement Type is %d", ndt_implement_type_tmp);
  try {
    ndt_ptr_ = getNDT<PointSource, PointTarget>(ndt_implement_type_);
    backup_ndt_ptr_ = getNDT<PointSource, PointTarget>(ndt_implement_type_);
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    const std::string s = fmt::format("DO NOT USE OMP FOR NOW (koji minoda)");
    throw std::runtime_error(s);
  } else if (ndt_implement_type_ == NDTImplementType::OMP_MULTI_VOXEL) {
    using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    std::cout << "KOJI Before: " << ndt_omp_ptr->getNumThreads() << std::endl;

    omp_params_.num_threads = this->declare_parameter("omp_num_threads", omp_params_.num_threads);
    omp_params_.num_threads = std::max(omp_params_.num_threads, 1);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
  }
  {
    using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    std::cout << "KOJI After: " << ndt_omp_ptr->getNumThreads() << std::endl;
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

  // copyNDT(ndt_ptr_, backup_ndt_ptr_, ndt_implement_type_);
  std::cout << "Before: " << backup_ndt_ptr_->getStepSize() << std::endl;
  backup_ndt_ptr_ = copyNDT(ndt_ptr_, ndt_implement_type_);
  std::cout << "After: " << backup_ndt_ptr_->getStepSize() << std::endl;

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    trans_epsilon, step_size, resolution, max_iterations);

  int converged_param_type_tmp = this->declare_parameter("converged_param_type", 0);
  converged_param_type_ = static_cast<ConvergedParamType>(converged_param_type_tmp);
  if (
    ndt_implement_type_ != NDTImplementType::OMP_MULTI_VOXEL &&
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

  // rclcpp::CallbackGroup::SharedPtr map_callback_group_;
  map_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  auto map_sub_opt = rclcpp::SubscriptionOptions();
  map_sub_opt.callback_group = map_callback_group_;

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 100,
    std::bind(&NDTScanMatcher::callbackInitialPose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  // map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "pointcloud_map", rclcpp::QoS{1},
  //   std::bind(&NDTScanMatcher::callbackMapPoints, this, std::placeholders::_1), map_sub_opt);
  // map_points_sub_ = this->create_subscription<autoware_map_msgs::msg::PCDMapArray>(
  //   "pointcloud_map", rclcpp::QoS{1},
  //   std::bind(&NDTScanMatcher::callbackMapPoints, this, std::placeholders::_1), map_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(points_queue_size),
    std::bind(&NDTScanMatcher::callbackSensorPoints, this, std::placeholders::_1), main_sub_opt);
  regularization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "regularization_pose_with_covariance", 100,
      std::bind(&NDTScanMatcher::callbackRegularizationPose, this, std::placeholders::_1));
  ekf_odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "ekf_odom", 100,
      std::bind(&NDTScanMatcher::callbackEKFOdom, this, std::placeholders::_1), main_sub_opt);

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

  koji_map_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/map/pointcloud_map/koji", rclcpp::QoS{1}.transient_local());

  service_ = this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(&NDTScanMatcher::serviceNDTAlign, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), map_callback_group_);
    // rclcpp::ServicesQoS().get_rmw_qos_profile());

  pcd_loader_client_ = this->create_client<autoware_map_msgs::srv::LoadPCDMapsGeneral>(
    "pcd_loader_service", rmw_qos_profile_services_default);
  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "Waiting for pcd loader service...");
  }
  last_update_position_ptr_ = nullptr;
  current_position_ptr_ = nullptr;

  double map_update_dt = 1.0;
  auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&NDTScanMatcher::mapUpdateTimerCallback, this), map_callback_group_);

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timerDiagnostic, this);
  diagnostic_thread_.detach();
}

void NDTScanMatcher::timerDiagnostic()
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

void NDTScanMatcher::serviceNDTAlign(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  std::cout << "=============== KOJI serviceNDTAlign called ===============" << std::endl;
  // (void)req; (void)res;
  ndt_service_align_in_progress_ = true;

  // get TF from pose_frame to map_frame
  auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(map_frame_, req->pose_with_covariance.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto mapTF_initial_pose_msg = transform(req->pose_with_covariance, *TF_pose_to_map_ptr);
  updateMap(mapTF_initial_pose_msg.pose.pose.position);

  if (ndt_ptr_->getInputSource() == nullptr) {
    res->success = false;
    res->seq = req->seq;
    RCLCPP_WARN(get_logger(), "No InputSource");
    return;
  }
  std::cout << "[KOJI serviceNDTAlign] finished updateMap, starting alignUsingMonteCarlo" << std::endl;;

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  res->pose_with_covariance = alignUsingMonteCarlo(ndt_ptr_, mapTF_initial_pose_msg);
  key_value_stdmap_["state"] = "Sleeping";
  res->success = true;
  res->seq = req->seq;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;

  last_update_position_ptr_ = std::make_shared<geometry_msgs::msg::Point>(res->pose_with_covariance.pose.pose.position);
  ndt_service_align_in_progress_ = false;

  std::cout << "=============== KOJI serviceNDTAlign Finished!!!! ===============" << std::endl;;
}

void NDTScanMatcher::callbackEKFOdom(nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr)
{
  current_position_ptr_ = std::make_shared<geometry_msgs::msg::Point>(odom_ptr->pose.pose.position);
  
  double distance = norm_xy(*current_position_ptr_, *last_update_position_ptr_);
  double LIDAR_CROP_DISTANCE = 100;
  // std::cout << "KOJI distance " << distance << std::endl;;
  if (distance + LIDAR_CROP_DISTANCE > dml_loading_radius_) {
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Dynamic map loading is not keeping up.");
  }
}

void NDTScanMatcher::mapUpdateTimerCallback()
{
  std::cout << "....................... KOJI timerCallback start!!!!!!!!! ..................." << std::endl;;

  if (current_position_ptr_ == nullptr) return;
  if (last_update_position_ptr_ == nullptr) return;
  if (ndt_service_align_in_progress_) return;

  std::cout << "....................... KOJI timerCallback running ..................." << std::endl;;

  // continue only if we should update the map
  if (shouldUpdateMap(*current_position_ptr_))
  {
    std::cout << "=============== KOJI timerCallback called ===============" << std::endl;;
    std::cout << "KOJI timerCallback (current_position.x, last pos.x) = (" << current_position_ptr_->x << ", " << last_update_position_ptr_->x << ")" << std::endl;
    updateMap(*current_position_ptr_);
    *last_update_position_ptr_ = *current_position_ptr_;
    std::cout << "=============== KOJI timerCallback Finished!!!! ===============" << std::endl;;
  }
}

bool NDTScanMatcher::shouldUpdateMap(const geometry_msgs::msg::Point & position)
{
  if (last_update_position_ptr_ == nullptr) return false;
  double distance = norm_xy(position, *last_update_position_ptr_);
  return distance > dml_update_map_distance_;
}

void NDTScanMatcher::updateMap(const geometry_msgs::msg::Point & position)
{
  // create a loading request with mode = 1
  auto request = std::make_shared<autoware_map_msgs::srv::LoadPCDMapsGeneral::Request>();
  request->mode = 1; // differential load
  request->area.center = position;
  request->area.radius = dml_loading_radius_;
  request->already_loaded_ids = getCurrentMapIDs(ndt_ptr_);

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request,
    [this](const rclcpp::Client<autoware_map_msgs::srv::LoadPCDMapsGeneral>::SharedFuture
            response) {
      (void)response;
      std::lock_guard<std::mutex> lock{pcd_loader_client_mutex_};
      value_ready_ = true;
      condition_.notify_all();
    })};
  std::unique_lock<std::mutex> lock{pcd_loader_client_mutex_};
  condition_.wait(lock, [this]() { return value_ready_; });

  // call callbackMapPoints()
  std::vector<std::string> map_ids_ndt_will_possess = std::vector<std::string>();
  for (const std::string & map_id: result.get()->already_loaded_ids) {
    map_ids_ndt_will_possess.push_back(map_id);
  }
  for (const auto & map_with_id: result.get()->loaded_pcds) {
    map_ids_ndt_will_possess.push_back(map_with_id.id);
  }
  std::vector<std::string> map_ids_to_remove = getMapIDsToRemove(map_ids_ndt_will_possess);

  updateNdtWithNewMap(result.get()->loaded_pcds, map_ids_to_remove);
}

std::vector<std::string> NDTScanMatcher::getMapIDsToRemove(
  const std::vector<std::string> & map_ids_ndt_will_possess)
{
  std::vector<std::string> map_ids_to_remove;
  std::vector<std::string> current_map_ids = getCurrentMapIDs(ndt_ptr_);
  for (std::string current_map_id: current_map_ids)
  {
    bool should_remove = std::find(map_ids_ndt_will_possess.begin(), map_ids_ndt_will_possess.end(), current_map_id) == map_ids_ndt_will_possess.end();
    if (!should_remove) continue;
    map_ids_to_remove.push_back(current_map_id);
  }
  return map_ids_to_remove;
}

std::vector<std::string> NDTScanMatcher::getCurrentMapIDs(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr)
{
  using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
  std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr);
  return ndt_omp_ptr->getCurrentMapIDs();
}

void NDTScanMatcher::updateNdtWithNewMap(
  const std::vector<autoware_map_msgs::msg::PCDMapWithID> & maps_to_add,
  const std::vector<std::string> & map_ids_to_remove)
{
  std::cout << "Received a map! add: " << int(maps_to_add.size()) << ", remove: " << int(map_ids_to_remove.size()) << std::endl;
  if ((int(maps_to_add.size()) == 0) & (int(map_ids_to_remove.size()) == 0)) {
    std::cout << "No map update" << std::endl;
    return;
  }
  const auto KOJI_exe_start_time = std::chrono::system_clock::now();

  if (ndt_implement_type_ == NDTImplementType::OMP_MULTI_VOXEL) {
    using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    std::shared_ptr<T> backup_ndt_omp_ptr = std::dynamic_pointer_cast<T>(backup_ndt_ptr_);

    // Add pcd
    for (const auto & map_to_add: maps_to_add) {
      pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
      pcl::fromROSMsg(map_to_add.pcd, *map_points_ptr);
      backup_ndt_omp_ptr->setInputTarget(map_points_ptr, map_to_add.id);
    }

    // Remove pcd
    for (const std::string map_id_to_remove: map_ids_to_remove) {
      backup_ndt_omp_ptr->removeTarget(map_id_to_remove);
    }

    backup_ndt_omp_ptr->createVoxelKdtree();
  }

  const auto KOJI_exe_end_time = std::chrono::system_clock::now();
  const double KOJI_exe_time = std::chrono::duration_cast<std::chrono::microseconds>(KOJI_exe_end_time - KOJI_exe_start_time).count() / 1000.0;
  RCLCPP_INFO_STREAM(get_logger(), "KOJI until align in callbackMapPoints @ndt_scan_matcher: " << KOJI_exe_time << " [ms]");

  // swap
  ndt_map_mtx_.lock();
  ndt_ptr_.swap(backup_ndt_ptr_);
  ndt_map_mtx_.unlock();

  publishPartialPCDMap();
  backup_ndt_ptr_ = copyNDT(ndt_ptr_, ndt_implement_type_);
}


void NDTScanMatcher::callbackInitialPose(
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
    getTransform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    auto mapTF_initial_pose_msg_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    *mapTF_initial_pose_msg_ptr = transform(*initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callbackRegularizationPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  regularization_pose_msg_ptr_array_.push_back(pose_conv_msg_ptr);
}

void NDTScanMatcher::publishPartialPCDMap()
{
  using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
  std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
  pcl::PointCloud<PointTarget> map_pcl = ndt_omp_ptr->getVoxelPCD();

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  koji_map_pub_->publish(map_msg);
  std::cout << "published a map!! size = " << map_msg.width << std::endl;
}

void NDTScanMatcher::callbackSensorPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  const std::string & sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const rclcpp::Time sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  auto TF_base_to_sensor_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);
  backup_ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr); // KOJI is this really the best way?
 
  // start of critical section for initial_pose_msg_ptr_array_
  std::unique_lock<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  // check
  if (initial_pose_msg_ptr_array_.size() <= 1) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No Pose!");
    return;
  }
  // searchNNPose using timestamp
  auto initial_pose_old_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  auto initial_pose_new_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getNearestTimeStampPose(
    initial_pose_msg_ptr_array_, sensor_ros_time, initial_pose_old_msg_ptr,
    initial_pose_new_msg_ptr);
  popOldPose(initial_pose_msg_ptr_array_, sensor_ros_time);

  // check the time stamp
  bool valid_old_timestamp = validateTimeStampDifference(
    initial_pose_old_msg_ptr->header.stamp, sensor_ros_time, initial_pose_timeout_sec_);
  bool valid_new_timestamp = validateTimeStampDifference(
    initial_pose_new_msg_ptr->header.stamp, sensor_ros_time, initial_pose_timeout_sec_);

  // check the position jumping (ex. immediately after the initial pose estimation)
  bool valid_new_to_old_distance = validatePositionDifference(
    initial_pose_old_msg_ptr->pose.pose.position, initial_pose_new_msg_ptr->pose.pose.position,
    initial_pose_distance_tolerance_m_);

  // must all validations are true
  if (!(valid_old_timestamp && valid_new_timestamp && valid_new_to_old_distance)) {
    RCLCPP_WARN(get_logger(), "Validation error.");
    return;
  }

  // If regularization is enabled and available, set pose to NDT for regularization
  if (regularization_enabled_ && (ndt_implement_type_ == NDTImplementType::OMP)) {
    ndt_ptr_->unsetRegularizationPose();
    std::optional<Eigen::Matrix4f> pose_opt = interpolateRegularizationPose(sensor_ros_time);
    if (pose_opt.has_value()) {
      ndt_ptr_->setRegularizationPose(pose_opt.value());
      RCLCPP_DEBUG_STREAM(get_logger(), "Regularization pose is set to NDT");
    }
  }

  const auto initial_pose_msg =
    interpolatePose(*initial_pose_old_msg_ptr, *initial_pose_new_msg_ptr, sensor_ros_time);

  // enf of critical section for initial_pose_msg_ptr_array_
  initial_pose_array_lock.unlock();

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_cov_msg;
  initial_pose_cov_msg.header = initial_pose_msg.header;
  initial_pose_cov_msg.pose.pose = initial_pose_msg.pose;

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No MAP!");
    return;
  }
  // align
  const Eigen::Affine3d initial_pose_affine = fromRosPoseToEigen(initial_pose_cov_msg.pose.pose);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  key_value_stdmap_["state"] = "Aligning";
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";

  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    result_pose_matrix_array = ndt_ptr_->getFinalTransformationArray();
  std::vector<geometry_msgs::msg::Pose> result_pose_msg_array;
  for (const auto & pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose_affine);
    result_pose_msg_array.push_back(pose_msg);
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  const float transform_probability = ndt_ptr_->getTransformationProbability();
  const float nearest_voxel_transformation_likelihood =
    ndt_ptr_->getNearestVoxelTransformationLikelihood();

  const int iteration_num = ndt_ptr_->getFinalNumIteration();

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
  bool is_ok_iteration_num = iteration_num < ndt_ptr_->getMaximumIterations() + 2;
  if (!is_ok_iteration_num) {
    RCLCPP_WARN(
      get_logger(),
      "The number of iterations has reached its upper limit. The number of iterations: %d, Limit: "
      "%d",
      iteration_num, ndt_ptr_->getMaximumIterations() + 2);
  }
  // RCLCPP_WARN(
  //   get_logger(),
  //   "The number of iterations OK. The number of iterations: %d",
  //   iteration_num);

  bool is_local_optimal_solution_oscillation = false;
  if (!is_ok_iteration_num) {
    is_local_optimal_solution_oscillation = isLocalOptimalSolutionOscillation(
      result_pose_matrix_array, oscillation_threshold_, inversion_vector_threshold_);
  }

  bool is_ok_converged_param = false;
  if (converged_param_type_ == ConvergedParamType::TRANSFORM_PROBABILITY) {
    is_ok_converged_param = transform_probability > converged_param_transform_probability_;
    if (!is_ok_converged_param) {
      // RCLCPP_WARN( // KOJI
      //   get_logger(), "Transform Probability is below the threshold. Score: %lf, Threshold: %lf",
      //   transform_probability, converged_param_transform_probability_);
    }
  } else if (converged_param_type_ == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    is_ok_converged_param = nearest_voxel_transformation_likelihood >
                            converged_param_nearest_voxel_transformation_likelihood_;
    if (!is_ok_converged_param) {
      // RCLCPP_WARN( // KOJI
      //   get_logger(),
      //   "Nearest Voxel Transform Probability is below the threshold. Score: %lf, Threshold: %lf",
      //   nearest_voxel_transformation_likelihood,
      //   converged_param_nearest_voxel_transformation_likelihood_);
    }
  } else {
    is_ok_converged_param = false;
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Unknown converged param type.");
  }

  bool is_converged = false;
  static size_t skipping_publish_num = 0;
  if (is_ok_iteration_num && is_ok_converged_param) {
    is_converged = true;
    skipping_publish_num = 0;
  } else {
    is_converged = false;
    ++skipping_publish_num;
    RCLCPP_WARN(get_logger(), "Not Converged");
  }

  // publish
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = output_pose_covariance_;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
    // current_position_ptr_ = std::make_shared<geometry_msgs::msg::Point>(result_pose_with_cov_msg.pose.pose.position); // KOJI
    // std::cout << "KOJI current.x = " << current_position_ptr_->x << std::endl;
  }

  publishTF(ndt_base_frame_, result_pose_stamped_msg);

  auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);

  initial_pose_with_covariance_pub_->publish(initial_pose_cov_msg);

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
  for (const auto & pose_msg : result_pose_msg_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = ExchangeColorCrc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }
  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = ExchangeColorCrc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);

  exe_time_pub_->publish(makeFloat32Stamped(sensor_ros_time, exe_time));

  transform_probability_pub_->publish(makeFloat32Stamped(sensor_ros_time, transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, nearest_voxel_transformation_likelihood));

  iteration_num_pub_->publish(makeInt32Stamped(sensor_ros_time, iteration_num));

  const float initial_to_result_distance =
    norm(initial_pose_cov_msg.pose.pose.position, result_pose_with_cov_msg.pose.pose.position);
  initial_to_result_distance_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance));

  const float initial_to_result_distance_old =
    norm(initial_pose_old_msg_ptr->pose.pose.position, result_pose_with_cov_msg.pose.pose.position);
  initial_to_result_distance_old_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance_old));

  const float initial_to_result_distance_new =
    norm(initial_pose_new_msg_ptr->pose.pose.position, result_pose_with_cov_msg.pose.pose.position);
  initial_to_result_distance_new_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance_new));

  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["nearest_voxel_transformation_likelihood"] =
    std::to_string(nearest_voxel_transformation_likelihood);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);
  if (is_local_optimal_solution_oscillation) {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "1";
  } else {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "0";
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses =
    createRandomPoseArray(initial_pose_with_cov, initial_estimate_particles_num_);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];

    const Eigen::Affine3d initial_pose_affine = fromRosPoseToEigen(initial_pose);
    const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
    ndt_ptr->align(*output_cloud, initial_pose_matrix);

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose result_pose = tf2::toMsg(result_pose_affine);

    const auto transform_probability = ndt_ptr->getTransformationProbability();
    const auto num_iteration = ndt_ptr->getFinalNumIteration();

    Particle particle(initial_pose, result_pose, transform_probability, num_iteration);
    particle_array.push_back(particle);
    const auto marker_array = makeDebugMarkers(
      this->now(), map_frame_, tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle, i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    const auto sensor_points_baselinkTF_ptr = ndt_ptr->getInputSource();
    pcl::transformPointCloud(
      *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = initial_pose_with_cov.header.stamp;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);
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

void NDTScanMatcher::publishTF(
  const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg)
{
  tf2_broadcaster_.sendTransform(tier4_autoware_utils::pose2transform(pose_msg, child_frame_id));
}

bool NDTScanMatcher::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr)
{
  const geometry_msgs::msg::TransformStamped identity =
    identityTransformStamped(this->now(), target_frame, source_frame);

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

bool NDTScanMatcher::validateTimeStampDifference(
  const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
  const double time_tolerance_sec)
{
  const double dt = std::abs((target_time - reference_time).seconds());
  if (dt > time_tolerance_sec) {
    RCLCPP_WARN(
      get_logger(),
      "Validation error. The reference time is %lf[sec], but the target time is %lf[sec]. The "
      "difference is %lf[sec] (the tolerance is %lf[sec]).",
      reference_time.seconds(), target_time.seconds(), dt, time_tolerance_sec);
    return false;
  }
  return true;
}

bool NDTScanMatcher::validatePositionDifference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m_)
{
  double distance = norm(target_point, reference_point);
  if (distance > distance_tolerance_m_) {
    RCLCPP_WARN(
      get_logger(),
      "Validation error. The distance from reference position to target position is %lf[m] (the "
      "tolerance is %lf[m]).",
      distance, distance_tolerance_m_);
    return false;
  }
  return true;
}

// bool NDTScanMatcher::hasCompatibleMap(
//   const geometry_msgs::msg::Point & initial_point)
// {
//   bool is_x_axis_ok = (initial_point.x > min_x_) && (initial_point.x < max_x_);
//   bool is_y_axis_ok = (initial_point.y > min_y_) && (initial_point.y < max_y_);

//   return is_x_axis_ok || is_y_axis_ok;
// }

std::optional<Eigen::Matrix4f> NDTScanMatcher::interpolateRegularizationPose(
  const rclcpp::Time & sensor_ros_time)
{
  if (regularization_pose_msg_ptr_array_.empty()) {
    return std::nullopt;
  }

  // synchronization
  auto regularization_old_msg_ptr =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  auto regularization_new_msg_ptr =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getNearestTimeStampPose(
    regularization_pose_msg_ptr_array_, sensor_ros_time, regularization_old_msg_ptr,
    regularization_new_msg_ptr);
  popOldPose(regularization_pose_msg_ptr_array_, sensor_ros_time);

  const geometry_msgs::msg::PoseStamped regularization_pose_msg =
    interpolatePose(*regularization_old_msg_ptr, *regularization_new_msg_ptr, sensor_ros_time);
  // if the interpolatePose fails, 0.0 is stored in the stamp
  if (rclcpp::Time(regularization_pose_msg.header.stamp).seconds() == 0.0) {
    return std::nullopt;
  }

  Eigen::Affine3d regularization_pose_affine;
  tf2::fromMsg(regularization_pose_msg.pose, regularization_pose_affine);
  return regularization_pose_affine.matrix().cast<float>();
}
