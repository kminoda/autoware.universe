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

#ifndef NDT__IMPL__PCL_GENERIC_HPP_
#define NDT__IMPL__PCL_GENERIC_HPP_

#include "ndt/pcl_generic.hpp"

#include <vector>

template <class PointSource, class PointTarget>
NormalDistributionsTransformPCLGeneric<
  PointSource, PointTarget>::NormalDistributionsTransformPCLGeneric()
: ndt_ptr_(new pcl::NormalDistributionsTransform<PointSource, PointTarget>)
{
}

template <class PointSource, class PointTarget>
NdtResult NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::align(
  const geometry_msgs::msg::Pose & initial_pose_msg)
{
  const Eigen::Matrix4f initial_pose_matrix =
    tier4_autoware_utils::poseToMatrix4f(initial_pose_msg);

  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    transformation_array_matrix = getFinalTransformationArray();
  std::vector<geometry_msgs::msg::Pose> transformation_array_msg;
  for (auto pose_matrix : transformation_array_matrix) {
    geometry_msgs::msg::Pose pose_ros = tier4_autoware_utils::matrix4fToPose(pose_matrix);
    transformation_array_msg.push_back(pose_ros);
  }

  NdtResult ndt_result;
  ndt_result.pose = tier4_autoware_utils::matrix4fToPose(getFinalTransformation());
  ndt_result.transformation_array = transformation_array_msg;
  ndt_result.transform_probability = getTransformationProbability();
  ndt_result.nearest_voxel_transformation_likelihood = getNearestVoxelTransformationLikelihood();
  ndt_result.iteration_num = getFinalNumIteration();
  return ndt_result;
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::setInputTarget(
  const pcl::shared_ptr<pcl::PointCloud<PointTarget>> & map_ptr)
{
  ndt_ptr_->setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::setInputSource(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & scan_ptr)
{
  ndt_ptr_->setInputSource(scan_ptr);
}

template <class PointSource, class PointTarget>
NDTImplementType NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getImplementationType()
{
  return NDTImplementType::PCL_GENERIC;
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::setMaximumIterations(
  int max_iter)
{
  ndt_ptr_->setMaximumIterations(max_iter);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::setResolution(float res)
{
  ndt_ptr_->setResolution(res);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::setStepSize(double step_size)
{
  ndt_ptr_->setStepSize(step_size);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::setTransformationEpsilon(
  double trans_eps)
{
  ndt_ptr_->setTransformationEpsilon(trans_eps);
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getMaximumIterations()
{
  return ndt_ptr_->getMaximumIterations();
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getFinalNumIteration() const
{
  return ndt_ptr_->getFinalNumIteration();
}

template <class PointSource, class PointTarget>
float NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getResolution() const
{
  return ndt_ptr_->getResolution();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getStepSize() const
{
  return ndt_ptr_->getStepSize();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getTransformationEpsilon()
{
  return ndt_ptr_->getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<
  PointSource, PointTarget>::getTransformationProbability() const
{
  return ndt_ptr_->getTransformationProbability();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<
  PointSource, PointTarget>::getNearestVoxelTransformationLikelihood() const
{
  // return ndt_ptr_->getNearestVoxelTransformationLikelihood();
  return 0.0;
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getFitnessScore()
{
  return ndt_ptr_->getFitnessScore();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<const pcl::PointCloud<PointTarget>>
NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getInputTarget() const
{
  return ndt_ptr_->getInputTarget();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<const pcl::PointCloud<PointSource>>
NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getInputSource() const
{
  return ndt_ptr_->getInputSource();
}

template <class PointSource, class PointTarget>
Eigen::Matrix4f
NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getFinalTransformation() const
{
  return ndt_ptr_->getFinalTransformation();
}

template <class PointSource, class PointTarget>
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getFinalTransformationArray()
  const
{
  return std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>();
}

template <class PointSource, class PointTarget>
Eigen::Matrix<double, 6, 6>
NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getHessian() const
{
  // return ndt_ptr_->getHessian();
  return Eigen::Matrix<double, 6, 6>();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<pcl::search::KdTree<PointTarget>>
NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::getSearchMethodTarget() const
{
  return ndt_ptr_->getSearchMethodTarget();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::
  calculateTransformationProbability(const pcl::PointCloud<PointSource> & trans_cloud) const
{
  (void)trans_cloud;
  // return ndt_ptr_->calculateTransformationProbability(trans_cloud);
  return 0.0;
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>::
  calculateNearestVoxelTransformationLikelihood(
    const pcl::PointCloud<PointSource> & trans_cloud) const
{
  (void)trans_cloud;
  // return ndt_ptr_->calculateNearestVoxelTransformationLikelihood(trans_cloud);
  return 0.0;
}

#endif  // NDT__IMPL__PCL_GENERIC_HPP_
