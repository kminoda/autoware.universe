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

#include "ndt_scan_matcher/ndt_scan_matching_module.hpp"

NdtScanMatchingModule::NdtScanMatchingModule()
{};

AlignResults NdtScanMatchingModule::setInputSensorPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
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
}