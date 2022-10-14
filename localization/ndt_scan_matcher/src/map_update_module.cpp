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

#include "ndt_scan_matcher/map_update_module.hpp"


template <typename T, typename U>
double norm_xy(const T p1, const U p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
  std::shared_ptr<NDTBase> * ndt_ptr,
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module,
  std::string map_frame,
  rclcpp::CallbackGroup::SharedPtr main_callback_group,
  rclcpp::CallbackGroup::SharedPtr map_callback_group,
  std::map<std::string, std::string> * key_value_stdmap_ptr)
: ndt_ptr_ptr_(ndt_ptr),
  ndt_ptr_mutex_(ndt_ptr_mutex),
  map_frame_(map_frame),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  tf2_listener_module_(tf2_listener_module),
  key_value_stdmap_ptr_(key_value_stdmap_ptr),
  dml_update_map_distance_(node->declare_parameter<double>("dml_update_map_distance")),
  dml_loading_radius_(node->declare_parameter<double>("dml_loading_radius"))
{
  initial_estimate_particles_num_ =
    node->declare_parameter<int>("initial_estimate_particles_num");

  sensor_aligned_pose_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("debug/monte_carlo_points_aligned", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  ekf_odom_sub_ =
    node->create_subscription<nav_msgs::msg::Odometry>(
      "ekf_odom", 100,
      std::bind(&MapUpdateModule::callback_ekf_odom, this, std::placeholders::_1), main_sub_opt);

  loaded_pcd_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  service_ = node->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(&MapUpdateModule::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), map_callback_group);

  pcd_loader_client_ = node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>(
    "pcd_loader_service", rmw_qos_profile_services_default);
  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Waiting for pcd loader service...");
  }
  last_update_position_ptr_ = nullptr;
  current_position_ptr_ = nullptr;

  double map_update_dt = 1.0;
  auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    node, clock_, period_ns, std::bind(&MapUpdateModule::map_update_timer_callback, this), map_callback_group);

  backup_ndt_ptr_ = get_ndt<PointSource, PointTarget>((*ndt_ptr_ptr_)->getImplementationType());
  backup_ndt_ptr_->setParam((*ndt_ptr_ptr_)->getParam());
  if ((*ndt_ptr_ptr_)->getImplementationType() == NDTImplementType::OMP_MULTI_VOXEL) {
    using NDT_OMP_MULTI_VOXEL = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    std::dynamic_pointer_cast<NDT_OMP_MULTI_VOXEL>(backup_ndt_ptr_)->setOMPMultiVoxelParam(
      std::dynamic_pointer_cast<NDT_OMP_MULTI_VOXEL>(*(ndt_ptr_ptr_))->getOMPMultiVoxelParam());
  }
}

void MapUpdateModule::service_ndt_align(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  RCLCPP_INFO(logger_, "Start updating NDT map (service_ndt_align)");

  // get TF from pose_frame to map_frame
  auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    clock_->now(), map_frame_, req->pose_with_covariance.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto mapTF_initial_pose_msg = transform(req->pose_with_covariance, *TF_pose_to_map_ptr);
  update_map(mapTF_initial_pose_msg.pose.pose.position);

  std::cout << "ndt_scan_matcher KOJI CHECK INPUT SOURCE.......... @ " << *ndt_ptr_ptr_ << std::endl;
  if ((*ndt_ptr_ptr_)->getInputSource() == nullptr) {
    res->success = false;
    RCLCPP_WARN(logger_, "No InputSource");
    return;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(*ndt_ptr_mutex_);

  (*key_value_stdmap_ptr_)["state"] = "Aligning";
  res->pose_with_covariance = align_using_monte_carlo(*ndt_ptr_ptr_, mapTF_initial_pose_msg);
  (*key_value_stdmap_ptr_)["state"] = "Sleeping";
  res->success = true;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;

  last_update_position_ptr_ = std::make_shared<geometry_msgs::msg::Point>(res->pose_with_covariance.pose.pose.position);
}

void MapUpdateModule::callback_ekf_odom(nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr)
{
  current_position_ptr_ = std::make_shared<geometry_msgs::msg::Point>(odom_ptr->pose.pose.position);
  
  if (last_update_position_ptr_ == nullptr) {return;}
  double distance = norm_xy(*current_position_ptr_, *last_update_position_ptr_);
  double LIDAR_CROP_DISTANCE = 100;
  if (distance + LIDAR_CROP_DISTANCE > dml_loading_radius_) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1, "Dynamic map loading is not keeping up.");
  }
}

void MapUpdateModule::map_update_timer_callback()
{
  if (current_position_ptr_ == nullptr) return;
  if (last_update_position_ptr_ == nullptr) return;

  // continue only if we should update the map
  if (should_update_map(*current_position_ptr_))
  {
    RCLCPP_INFO(logger_, "Start updating NDT map (timer_callback)");
    update_map(*current_position_ptr_);
    *last_update_position_ptr_ = *current_position_ptr_;
  }
}

bool MapUpdateModule::should_update_map(const geometry_msgs::msg::Point & position)
{
  if (last_update_position_ptr_ == nullptr) return false;
  double distance = norm_xy(position, *last_update_position_ptr_);
  return distance > dml_update_map_distance_;
}

void MapUpdateModule::update_map(const geometry_msgs::msg::Point & position)
{
  // create a loading request with mode = 1
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
  request->area.center = position;
  request->area.radius = dml_loading_radius_;
  request->cached_ids = get_current_map_ids(*ndt_ptr_ptr_);

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request,
    [this](const rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture
            response) {
      (void)response;
      std::lock_guard<std::mutex> lock{pcd_loader_client_mutex_};
      value_ready_ = true;
      condition_.notify_all();
    })};
  std::unique_lock<std::mutex> lock{pcd_loader_client_mutex_};
  condition_.wait(lock, [this]() { return value_ready_; });

  update_ndt_with_new_map(result.get()->new_pointcloud_with_ids, result.get()->ids_to_remove);
}

std::vector<std::string> MapUpdateModule::get_current_map_ids(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr)
{
  using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
  std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr);
  return ndt_omp_ptr->getCurrentMapIDs();
}

void MapUpdateModule::update_ndt_with_new_map(
  const std::vector<autoware_map_msgs::msg::PointCloudMapCellWithID> & maps_to_add,
  const std::vector<std::string> & map_ids_to_remove)
{
  RCLCPP_INFO(logger_, "Update map (Add: %d, Remove: %d)", int(maps_to_add.size()), int(map_ids_to_remove.size()));
  if ((int(maps_to_add.size()) == 0) & (int(map_ids_to_remove.size()) == 0)) {
    RCLCPP_INFO(logger_, "Skip map update");
    return;
  }
  const auto exe_start_time = std::chrono::system_clock::now();

  if ((*ndt_ptr_ptr_)->getImplementationType() == NDTImplementType::OMP_MULTI_VOXEL) {
    using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
    std::shared_ptr<T> backup_ndt_omp_ptr = std::dynamic_pointer_cast<T>(backup_ndt_ptr_);

    backup_ndt_omp_ptr->setInputSource(
      std::dynamic_pointer_cast<T>(*ndt_ptr_ptr_)->getInputSourceTempKOJI());

    // Add pcd
    for (const auto & map_to_add: maps_to_add) {
      pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
      pcl::fromROSMsg(map_to_add.pointcloud, *map_points_ptr);
      backup_ndt_omp_ptr->setInputTarget(map_points_ptr, map_to_add.cell_id);
    }

    // Remove pcd
    for (const std::string & map_id_to_remove: map_ids_to_remove) {
      backup_ndt_omp_ptr->removeTarget(map_id_to_remove);
    }

    backup_ndt_omp_ptr->createVoxelKdtree();
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;
  RCLCPP_INFO(logger_, "Time duration for creating new ndt_ptr: %f [ms]", exe_time);

  // swap
  (*ndt_ptr_mutex_).lock();
  (*ndt_ptr_ptr_).swap(backup_ndt_ptr_);
  (*ndt_ptr_mutex_).unlock();

  publish_partial_pcd_map();

  // TODO (koji minoda): Any way to simplify this copy part?
  using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
  backup_ndt_ptr_ = std::make_shared<T>(*std::dynamic_pointer_cast<T>(*ndt_ptr_ptr_));
}

geometry_msgs::msg::PoseWithCovarianceStamped MapUpdateModule::align_using_monte_carlo(
  const std::shared_ptr<NDTBase> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(logger_, "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses =
    create_random_pose_array(initial_pose_with_cov, initial_estimate_particles_num_);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];
    const NdtResult ndt_result = ndt_ptr->align(initial_pose);

    Particle particle(
      initial_pose, ndt_result.pose, ndt_result.transform_probability, ndt_result.iteration_num);
    particle_array.push_back(particle);
    const auto marker_array = make_debug_markers(
      clock_->now(), map_frame_, tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle, i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    pcl::transformPointCloud(
      *ndt_ptr->getInputSource(), *sensor_points_mapTF_ptr, tier4_autoware_utils::poseToMatrix4f(ndt_result.pose));
    publish_point_cloud(initial_pose_with_cov.header.stamp, map_frame_, sensor_points_mapTF_ptr);
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

void MapUpdateModule::publish_partial_pcd_map()
{
  using T = NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>;
  std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(*(ndt_ptr_ptr_));
  pcl::PointCloud<PointTarget> map_pcl = ndt_omp_ptr->getVoxelPCD();

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}

void MapUpdateModule::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const std::shared_ptr<const pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);
}
