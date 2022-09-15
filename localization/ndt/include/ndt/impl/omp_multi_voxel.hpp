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

#ifndef NDT__IMPL__OMP_MULTI_VOXEL_HPP_
#define NDT__IMPL__OMP_MULTI_VOXEL_HPP_

#include "ndt/omp_multi_voxel.hpp"

#include <vector>

template <class PointSource, class PointTarget>
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::NormalDistributionsTransformOMPMultiVoxel()
: ndt_ptr_(new pclomp::NormalDistributionsTransformMultiVoxel<PointSource, PointTarget>)
{
}

template <class PointSource, class PointTarget>
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::NormalDistributionsTransformOMPMultiVoxel(
  const NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget> &obj)
{
  using T = pclomp::NormalDistributionsTransformMultiVoxel<PointSource, PointTarget>;
  ndt_ptr_ = pcl::make_shared<T>(*obj.getNDTPtr());
}

template <class PointSource, class PointTarget>
NdtResult NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::align(
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
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setInputTarget(
  const pcl::shared_ptr<pcl::PointCloud<PointTarget>> & map_ptr, const std::string map_id)
{
  ndt_ptr_->setInputTarget(map_ptr, map_id);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setInputTarget(
  const pcl::shared_ptr<pcl::PointCloud<PointTarget>> & map_ptr)
{
  (void)map_ptr; // TODO fix this as this is very dangerous (koji minoda)
  // ndt_ptr_->setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::removeTarget(
  const std::string map_id)
{
  ndt_ptr_->removeTarget(map_id);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::createVoxelKdtree()
{
  ndt_ptr_->createVoxelKdtree();
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setInputSource(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & scan_ptr)
{
  ndt_ptr_->setInputSource(scan_ptr);
  scan_ptr_ = scan_ptr;
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<pcl::PointCloud<PointSource>>
  NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getInputSourceTempKOJI()
{
  return scan_ptr_;
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setMaximumIterations(int max_iter)
{
  ndt_ptr_->setMaximumIterations(max_iter);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setResolution(float res)
{
  ndt_ptr_->setResolution(res);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setStepSize(double step_size)
{
  ndt_ptr_->setStepSize(step_size);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setTransformationEpsilon(
  double trans_eps)
{
  ndt_ptr_->setTransformationEpsilon(trans_eps);
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getMaximumIterations()
{
  return ndt_ptr_->getMaximumIterations();
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getFinalNumIteration() const
{
  return ndt_ptr_->getFinalNumIteration();
}

template <class PointSource, class PointTarget>
float NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getResolution() const
{
  return ndt_ptr_->getResolution();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getStepSize() const
{
  return ndt_ptr_->getStepSize();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getTransformationEpsilon()
{
  return ndt_ptr_->getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getTransformationProbability()
  const
{
  return ndt_ptr_->getTransformationProbability();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMPMultiVoxel<
  PointSource, PointTarget>::getNearestVoxelTransformationLikelihood() const
{
  return ndt_ptr_->getNearestVoxelTransformationLikelihood();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getFitnessScore()
{
  return ndt_ptr_->getFitnessScore();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<const pcl::PointCloud<PointTarget>>
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getInputTarget() const
{
  return ndt_ptr_->getInputTarget();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<const pcl::PointCloud<PointSource>>
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getInputSource() const
{
  return ndt_ptr_->getInputSource();
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setOMPMultiVoxelParam(
  const OMPMultiVoxelParam & param)
{
  setNumThreads(param.num_threads);
}

template <class PointSource, class PointTarget>
typename NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::OMPMultiVoxelParam
  NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getOMPMultiVoxelParam()
{
  OMPMultiVoxelParam param;
  param.num_threads = getNumThreads();
  return param;
}

template <class PointSource, class PointTarget>
NDTImplementType NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getImplementationType()
{
  return NDTImplementType::OMP_MULTI_VOXEL;
}

template <class PointSource, class PointTarget>
Eigen::Matrix4f NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getFinalTransformation()
  const
{
  return ndt_ptr_->getFinalTransformation();
}

template <class PointSource, class PointTarget>
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getFinalTransformationArray() const
{
  return ndt_ptr_->getFinalTransformationArray();
}

template <class PointSource, class PointTarget>
Eigen::Matrix<double, 6, 6> NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getHessian()
  const
{
  // return ndt_ptr_->getHessian();
  return Eigen::Matrix<double, 6, 6>();
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setRegularizationScaleFactor(
  const float regularization_scale_factor)
{
  ndt_ptr_->setRegularizationScaleFactor(regularization_scale_factor);
}

template <class PointSource, class PointTarget>
float NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getRegularizationScaleFactor()
{
  return ndt_ptr_->getRegularizationScaleFactor();
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setRegularizationPose(
  const Eigen::Matrix4f & regularization_pose)
{
  ndt_ptr_->setRegularizationPose(regularization_pose);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::unsetRegularizationPose()
{
  ndt_ptr_->unsetRegularizationPose();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<pcl::search::KdTree<PointTarget>>
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getSearchMethodTarget() const
{
  return ndt_ptr_->getSearchMethodTarget();
}

template <class PointSource, class PointTarget>
double
NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::calculateTransformationProbability(
  const pcl::PointCloud<PointSource> & trans_cloud) const
{
  return ndt_ptr_->calculateTransformationProbability(trans_cloud);
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::
  calculateNearestVoxelTransformationLikelihood(
    const pcl::PointCloud<PointSource> & trans_cloud) const
{
  return ndt_ptr_->calculateNearestVoxelTransformationLikelihood(trans_cloud);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setNumThreads(int n)
{
  ndt_ptr_->setNumThreads(n);
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getNumThreads() const
{
  return ndt_ptr_->getNumThreads();
}

template <class PointSource, class PointTarget>
pcl::PointCloud<PointTarget> NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getVoxelPCD() const
{
  return ndt_ptr_->getVoxelPCD();
}

template <class PointSource, class PointTarget>
pcl::shared_ptr<pclomp::NormalDistributionsTransformMultiVoxel<PointSource, PointTarget>> 
  NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getNDTPtr() const
{
  return ndt_ptr_;
}

template <class PointSource, class PointTarget>
std::vector<std::string> NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getCurrentMapIDs() const
{
  return ndt_ptr_->getCurrentMapIDs();
}

#endif  // NDT__IMPL__OMP_MULTI_VOXEL_HPP_
