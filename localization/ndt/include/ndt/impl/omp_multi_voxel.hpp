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
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::align(
  pcl::PointCloud<PointSource> & output, const Eigen::Matrix4f & guess)
{
  ndt_ptr_->align(output, guess);
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

// template <class PointSource, class PointTarget>
// void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::setNeighborhoodSearchMethod(
//   pclomp::NeighborSearchMethod method)
// {
//   ndt_ptr_->setNeighborhoodSearchMethod(method);
// }

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getNumThreads() const
{
  return ndt_ptr_->getNumThreads();
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getVoxelPCD(
  pcl::PointCloud<PointTarget> & output)
{
  ndt_ptr_->getVoxelPCD(output);
}

// template <class PointSource, class PointTarget>
// pclomp::NeighborSearchMethod
// NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getNeighborhoodSearchMethod() const
// {
//   return ndt_ptr_->getNeighborhoodSearchMethod();
// }

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::getNDTPtr(
  pcl::shared_ptr<pclomp::NormalDistributionsTransformMultiVoxel<PointSource, PointTarget>> & output) const
{
  output = ndt_ptr_;
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget>::copyFrom(
  const NormalDistributionsTransformOMPMultiVoxel<PointSource, PointTarget> & input)
{
  pcl::shared_ptr<pclomp::NormalDistributionsTransformMultiVoxel<PointSource, PointTarget>> ndt_ptr_input;
  input.getNDTPtr(ndt_ptr_input);
  ndt_ptr_->copyFrom(*ndt_ptr_input);
  ndt_ptr_->setNumThreads(ndt_ptr_input->getNumThreads());
  ndt_ptr_->setTransformationEpsilon(ndt_ptr_input->getTransformationEpsilon());
  ndt_ptr_->setStepSize(ndt_ptr_input->getStepSize());
  ndt_ptr_->setResolution(ndt_ptr_input->getResolution());
  ndt_ptr_->setMaximumIterations(ndt_ptr_input->getMaximumIterations());
}


#endif  // NDT__IMPL__OMP_MULTI_VOXEL_HPP_
