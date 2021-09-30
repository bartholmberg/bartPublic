## Evaluate Ceres For Open 3d Registration
Currently **open 3d** uses the native eigen solver for icp.  K4a point clouds register with
open3d eigen just fine, but **only for about 3 degrees [sometime 4-6 degrees] of rotation** . Currently o3d does not use gravity vector to compensate pitch/roll and so registration search may be not be as  robust as possible.  Also, not clear if the eigen solver can use parameter constraints directly. 

Ceres solver is well known, and has equality constraints for the parameters. Also there is a reliable
gravity🧲 vector for **k4a** IMU ☄ and can use it to correct for pitch/roll.  Using ceres solver can reduce the the parameter search space to 4 translation [x,y,z] and yaw [ rotation around y] and potentially extend the max rotation tolerated for the next point cloud.


![ICP picture](C:\\repo\\bart\\ProjectSuraNovi\\sow_d1.gif)

Note* that k4a imu has  z and y axis swapped compared to the depth/camera coordinates and open 3d. 

### work 🚣 items

####    **Core Ceres In O3d tasks**


1. *eigen solver.* o3d solver current.  o3d *correspondence* algorithm. 
   - can we bring correspondence set generator out to python api
   - is correspondence source/target same as target/source.  Can we use this as a filter
   - 💂‍♀️
2. 
3. *eigen solver*. Is it possible to add parameter (pitch/roll) constraints directly to eigen solver.  
4. *ceres solver*. 
   
   - Do example with constraints
   - Ceres use in Cartographer.  cartographer implementation ,
     Look at this test used for cart.  [cartographer: ceres_scan_matcher_3d_test.cc](http://docs.ros.org/en/indigo/api/cartographer/html/ceres__scan__matcher__3d__test_8cc_source.html)








## [ceres tutorial](http://ceres-solver.org/tutorial.html)

```python
$ def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result
```

<img src="C:\Users\barth\OneDrive\Desktop\33672.jpg" style="zoom:10%;" />
    



Consider the problem of fitting the following curve ([Rat43](http://www.itl.nist.gov/div898/strd/nls/data/ratkowsky3.shtml)) to data:

$$y = \frac{b_1}{(1+e^{b_2-b_3x})^{1/b_4}}$$

That is, given some data $\{x_i, y_i\},\ \forall i=1,... ,n$, determine parameters $b_1, b_2, b_3$ and $b_4$ that best fit this data.

That is, given some data $\{z_k, g_k\},\ \exists k=1,... ,m$, determine parameters $p_1, p_2, p_3$, and  $xx_1,p_4$  that best fit this data.

###Which can be stated as the problem of finding the values of $b_1, b_2, b_3$ and $b_4$ are the ones that minimize the following objective function[^1]:

$$

```c++
TESTF( CeresScanMatcher3DTest, PerfectEstimate) {
TestFromInitialPose(       			transform::Rigid3d::Translation(Eigen::Vector3d(-1., 0., 0.)));}
```
* 


### ceres cost function 



```c++

/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {

// Penalizes differences between IMU data and optimized accelerations.
class AccelerationCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const double scaling_factor,
      const Eigen::Vector3d& delta_velocity_imu_frame,
      const double first_delta_time_seconds,
      const double second_delta_time_seconds) {
    return new ceres::AutoDiffCostFunction<
        AccelerationCostFunction3D, 3 /* residuals */,
        4 /* rotation variables */, 3 /* position variables */,
        3 /* position variables */, 3 /* position variables */,
        1 /* gravity variables */, 4 /* rotation variables */>(
        new AccelerationCostFunction3D(scaling_factor, delta_velocity_imu_frame, first_delta_time_seconds,
                                       second_delta_time_seconds));
  }

  template <typename T>
  bool operator()(const T* const middle_rotation, const T* const start_position,
                  const T* const middle_position, const T* const end_position,
                  const T* const gravity_constant,
                  const T* const imu_calibration, T* residual) const {
    const Eigen::Quaternion<T> eigen_imu_calibration(
        imu_calibration[0], imu_calibration[1], imu_calibration[2],
        imu_calibration[3]);
    const Eigen::Matrix<T, 3, 1> imu_delta_velocity =
        ToEigen(middle_rotation) * eigen_imu_calibration *
            delta_velocity_imu_frame_.cast<T>() -
        *gravity_constant *
            (0.5 * (first_delta_time_seconds_ + second_delta_time_seconds_) *
             Eigen::Vector3d::UnitZ())
                .cast<T>();
    const Eigen::Matrix<T, 3, 1> start_velocity =
        (Eigen::Map<const Eigen::Matrix<T, 3, 1>>(middle_position) -
         Eigen::Map<const Eigen::Matrix<T, 3, 1>>(start_position)) /
        T(first_delta_time_seconds_);
    const Eigen::Matrix<T, 3, 1> end_velocity =
        (Eigen::Map<const Eigen::Matrix<T, 3, 1>>(end_position) -
         Eigen::Map<const Eigen::Matrix<T, 3, 1>>(middle_position)) /
        T(second_delta_time_seconds_);
    const Eigen::Matrix<T, 3, 1> delta_velocity = end_velocity - start_velocity;

    (Eigen::Map<Eigen::Matrix<T, 3, 1>>(residual) =
         T(scaling_factor_) * (imu_delta_velocity - delta_velocity));
    return true;
  }

 private:
  AccelerationCostFunction3D(const double scaling_factor,
                             const Eigen::Vector3d& delta_velocity_imu_frame,
                             const double first_delta_time_seconds,
                             const double second_delta_time_seconds)
      : scaling_factor_(scaling_factor),
        delta_velocity_imu_frame_(delta_velocity_imu_frame),
        first_delta_time_seconds_(first_delta_time_seconds),
        second_delta_time_seconds_(second_delta_time_seconds) {}

  AccelerationCostFunction3D(const AccelerationCostFunction3D&) = delete;
  AccelerationCostFunction3D& operator=(const AccelerationCostFunction3D&) =
      delete;

  template <typename T>
  static Eigen::Quaternion<T> ToEigen(const T* const quaternion) {
    return Eigen::Quaternion<T>(quaternion[0], quaternion[1], quaternion[2],
                                quaternion[3]);
  }

  const double scaling_factor_;
  const Eigen::Vector3d delta_velocity_imu_frame_;
  const double first_delta_time_seconds_;
  const double second_delta_time_seconds_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ACCELERATION_COST_FUNCTION_3D_H_
```
### cartographer /ceres optimization function
```c++


#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/math.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/imu_integration.h"
#include "cartographer/mapping/internal/3d/rotation_parameterization.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/acceleration_cost_function_3d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_3d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/rotation_cost_function_3d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_3d.h"
#include "cartographer/transform/timestamped_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;

// For odometry.
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::OdometryData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

// For fixed frame pose.
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::FixedFramePoseData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id) ||
      !it->pose.has_value()) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose.value());
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  if (prev_it->pose.has_value()) {
    return absl::make_unique<transform::Rigid3d>(
        Interpolate(transform::TimestampedTransform{prev_it->time,
                                                    prev_it->pose.value()},
                    transform::TimestampedTransform{it->time, it->pose.value()},
                    time)
            .transform);
  }
  return nullptr;
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec3D& prev_node, const NodeSpec3D& next_node,
    const CeresPose& prev_node_pose, const CeresPose& next_node_pose) {
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);

  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation = InterpolateNodes3D(
          prev_node_pose.rotation(), prev_node_pose.translation(),
          next_node_pose.rotation(), next_node_pose.translation(),
          interpolation_parameter);
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}

void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeSpec3D>& node_data,
    MapById<NodeId, CeresPose>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem,
    double huber_scale) {
  for (const auto& landmark_node : landmark_nodes) {
    // Do not use landmarks that were not optimized for localization.
    for (const auto& observation : landmark_node.second.landmark_observations) {
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      CeresPose* prev_node_pose = &C_nodes->at(prev->id);
      CeresPose* next_node_pose = &C_nodes->at(next->id);
      if (!C_landmarks->count(landmark_id)) {
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      absl::make_unique<ceres::QuaternionParameterization>(),
                      problem));
        // Set landmark constant if it is frozen.
        if (landmark_node.second.frozen) {
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).translation());
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).rotation());
        }
      }
      problem->AddResidualBlock(
          LandmarkCostFunction3D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          new ceres::HuberLoss(huber_scale), prev_node_pose->rotation(),
          prev_node_pose->translation(), next_node_pose->rotation(),
          next_node_pose->translation(),
          C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
  }
}

}  // namespace

OptimizationProblem3D::OptimizationProblem3D(
    const optimization::proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem3D::~OptimizationProblem3D() {}

void OptimizationProblem3D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  imu_data_.Append(trajectory_id, imu_data);
}

void OptimizationProblem3D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

void OptimizationProblem3D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}

void OptimizationProblem3D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec3D& node_data) {
  node_data_.Append(trajectory_id, node_data);
  trajectory_data_[trajectory_id];
}

void OptimizationProblem3D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}

void OptimizationProblem3D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec3D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}

void OptimizationProblem3D::TrimTrajectoryNode(const NodeId& node_id) {
  imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
  if (node_data_.SizeOfTrajectoryOrZero(node_id.trajectory_id) == 0) {
    trajectory_data_.erase(node_id.trajectory_id);
  }
}

void OptimizationProblem3D::AddSubmap(
    const int trajectory_id, const transform::Rigid3d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec3D{global_submap_pose});
}

void OptimizationProblem3D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid3d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec3D{global_submap_pose});
}

void OptimizationProblem3D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void OptimizationProblem3D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void OptimizationProblem3D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  const auto translation_parameterization =
      [this]() -> std::unique_ptr<ceres::LocalParameterization> {
    return options_.fix_z_in_3d()
               ? absl::make_unique<ceres::SubsetParameterization>(
                     3, std::vector<int>{2})
               : nullptr;
  };

  // Set the starting point.
  CHECK(!submap_data_.empty());
  MapById<SubmapId, CeresPose> C_submaps;
  MapById<NodeId, CeresPose> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    if (first_submap) {
      first_submap = false;
      // Fix the first submap of the first trajectory except for allowing
      // gravity alignment.
      std::cout << "--------------blah blah got here" << std::endl;
      std::cout << "--------------blah blah and here too" << std::endl;
      //BAH, 4/3/2021
      //typedef ceres::AutoDiffLocalParameterization<ConstantYawQuaternionPlus, 4, 2> fooT0;
      typedef ceres::AutoDiffLocalParameterization<ConstantYawQuaternionPlus, 4, 3> fooT0;
      //typedef std::unique_ptr<fooT0> fooT1;
      std::unique_ptr<ceres::LocalParameterization> foo= std::make_unique<fooT0>();
      static auto  tptr = foo.get();
      auto foo2 = CeresPose(submap_id_data.data.global_pose,
                            translation_parameterization(),std::move(foo), &problem);
      C_submaps.Insert(submap_id_data.id,foo2);
      std::cout << "address of:" << std::hex<<tptr << std::endl;
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).translation());
    } else {
      C_submaps.Insert(
          submap_id_data.id,
          CeresPose(submap_id_data.data.global_pose,
                    translation_parameterization(),
                    absl::make_unique<ceres::QuaternionParameterization>(),
                    &problem));
    }
    if (frozen) {
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).rotation());
      problem.SetParameterBlockConstant(
          C_submaps.at(submap_id_data.id).translation());
    }
  }
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(
        node_id_data.id,
        CeresPose(node_id_data.data.global_pose, translation_parameterization(),
                  absl::make_unique<ceres::QuaternionParameterization>(),
                  &problem));
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).rotation());
      problem.SetParameterBlockConstant(
          C_nodes.at(node_id_data.id).translation());
    }
  }
  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        SpaCostFunction3D::CreateAutoDiffCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr /* loss function */,
        C_submaps.at(constraint.submap_id).rotation(),
        C_submaps.at(constraint.submap_id).translation(),
        C_nodes.at(constraint.node_id).rotation(),
        C_nodes.at(constraint.node_id).translation());
  }
  // Add cost functions for landmarks.
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem, options_.huber_scale());
  // Add constraints based on IMU observations of angular velocities and
  // linear acceleration.
  if (!options_.fix_z_in_3d()) {
    for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
      const int trajectory_id = node_it->id.trajectory_id;
      const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
      if (frozen_trajectories.count(trajectory_id) != 0) {
        // We skip frozen trajectories.
        node_it = trajectory_end;
        continue;
      }
      TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);

      problem.AddParameterBlock(trajectory_data.imu_calibration.data(), 4,
                                new ceres::QuaternionParameterization());
      if (!options_.use_online_imu_extrinsics_in_3d()) {
        problem.SetParameterBlockConstant(
            trajectory_data.imu_calibration.data());
      }
      CHECK(imu_data_.HasTrajectory(trajectory_id));
      const auto imu_data = imu_data_.trajectory(trajectory_id);
      CHECK(imu_data.begin() != imu_data.end());

      auto imu_it = imu_data.begin();
      auto prev_node_it = node_it;
      for (++node_it; node_it != trajectory_end; ++node_it) {
        const NodeId first_node_id = prev_node_it->id;
        const NodeSpec3D& first_node_data = prev_node_it->data;
        prev_node_it = node_it;
        const NodeId second_node_id = node_it->id;
        const NodeSpec3D& second_node_data = node_it->data;

        if (second_node_id.node_index != first_node_id.node_index + 1) {
          continue;
        }

        // Skip IMU data before the node.
        while (std::next(imu_it) != imu_data.end() &&
               std::next(imu_it)->time <= first_node_data.time) {
          ++imu_it;
        }

        auto imu_it2 = imu_it;
        const IntegrateImuResult<double> result = IntegrateImu(
            imu_data, first_node_data.time, second_node_data.time, &imu_it);
        const auto next_node_it = std::next(node_it);
        const common::Time first_time = first_node_data.time;
        const common::Time second_time = second_node_data.time;
        const common::Duration first_duration = second_time - first_time;
        if (next_node_it != trajectory_end &&
            next_node_it->id.node_index == second_node_id.node_index + 1) {
          const NodeId third_node_id = next_node_it->id;
          const NodeSpec3D& third_node_data = next_node_it->data;
          const common::Time third_time = third_node_data.time;
          const common::Duration second_duration = third_time - second_time;
          const common::Time first_center = first_time + first_duration / 2;
          const common::Time second_center = second_time + second_duration / 2;
          const IntegrateImuResult<double> result_to_first_center =
              IntegrateImu(imu_data, first_time, first_center, &imu_it2);
          const IntegrateImuResult<double> result_center_to_center =
              IntegrateImu(imu_data, first_center, second_center, &imu_it2);
          // 'delta_velocity' is the change in velocity from the point in time
          // halfway between the first and second poses to halfway between
          // second and third pose. It is computed from IMU data and still
          // contains a delta due to gravity. The orientation of this vector is
          // in the IMU frame at the second pose.
          const Eigen::Vector3d delta_velocity =
              (result.delta_rotation.inverse() *
               result_to_first_center.delta_rotation) *
              result_center_to_center.delta_velocity;
          problem.AddResidualBlock(
              AccelerationCostFunction3D::CreateAutoDiffCostFunction(
                  options_.acceleration_weight() /
                      common::ToSeconds(first_duration + second_duration),
                  delta_velocity, common::ToSeconds(first_duration),
                  common::ToSeconds(second_duration)),
              nullptr /* loss function */,
              C_nodes.at(second_node_id).rotation(),
              C_nodes.at(first_node_id).translation(),
              C_nodes.at(second_node_id).translation(),
              C_nodes.at(third_node_id).translation(),
              &trajectory_data.gravity_constant,
              trajectory_data.imu_calibration.data());
        }
        problem.AddResidualBlock(
            RotationCostFunction3D::CreateAutoDiffCostFunction(
                options_.rotation_weight() / common::ToSeconds(first_duration),
                result.delta_rotation),
            nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
            C_nodes.at(second_node_id).rotation(),
            trajectory_data.imu_calibration.data());
      }

      // Force gravity constant to be positive.
      problem.SetParameterLowerBound(&trajectory_data.gravity_constant, 0, 0.0);
    }
  }

  if (options_.fix_z_in_3d()) {
    // Add penalties for violating odometry (if available) and changes between
    // consecutive nodes.
    for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
      const int trajectory_id = node_it->id.trajectory_id;
      const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
      if (frozen_trajectories.count(trajectory_id) != 0) {
        node_it = trajectory_end;
        continue;
      }

      auto prev_node_it = node_it;
      for (++node_it; node_it != trajectory_end; ++node_it) {
        const NodeId first_node_id = prev_node_it->id;
        const NodeSpec3D& first_node_data = prev_node_it->data;
        prev_node_it = node_it;
        const NodeId second_node_id = node_it->id;
        const NodeSpec3D& second_node_data = node_it->data;

        if (second_node_id.node_index != first_node_id.node_index + 1) {
          continue;
        }

        // Add a relative pose constraint based on the odometry (if available).
        const std::unique_ptr<transform::Rigid3d> relative_odometry =
            CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                          second_node_data);
        if (relative_odometry != nullptr) {
          problem.AddResidualBlock(
              SpaCostFunction3D::CreateAutoDiffCostFunction(Constraint::Pose{
                  *relative_odometry, options_.odometry_translation_weight(),
                  options_.odometry_rotation_weight()}),
              nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
              C_nodes.at(first_node_id).translation(),
              C_nodes.at(second_node_id).rotation(),
              C_nodes.at(second_node_id).translation());
        }

        // Add a relative pose constraint based on consecutive local SLAM poses.
        const transform::Rigid3d relative_local_slam_pose =
            first_node_data.local_pose.inverse() * second_node_data.local_pose;
        problem.AddResidualBlock(
            SpaCostFunction3D::CreateAutoDiffCostFunction(
                Constraint::Pose{relative_local_slam_pose,
                                 options_.local_slam_pose_translation_weight(),
                                 options_.local_slam_pose_rotation_weight()}),
            nullptr /* loss function */, C_nodes.at(first_node_id).rotation(),
            C_nodes.at(first_node_id).translation(),
            C_nodes.at(second_node_id).rotation(),
            C_nodes.at(second_node_id).translation());
      }
    }
  }

  // Add fixed frame pose constraints.
  std::map<int, CeresPose> C_fixed_frames;
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    const TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    bool fixed_frame_pose_initialized = false;
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec3D& node_data = node_it->data;

      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      if (fixed_frame_pose == nullptr) {
        continue;
      }

      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      if (!fixed_frame_pose_initialized) {
        transform::Rigid3d fixed_frame_pose_in_map;
        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map =
              trajectory_data.fixed_frame_origin_in_map.value();
        } else {
          fixed_frame_pose_in_map =
              node_data.global_pose * constraint_pose.zbar_ij.inverse();
        }
        C_fixed_frames.emplace(
            std::piecewise_construct, std::forward_as_tuple(trajectory_id),
            std::forward_as_tuple(
                transform::Rigid3d(
                    fixed_frame_pose_in_map.translation(),
                    Eigen::AngleAxisd(
                        transform::GetYaw(fixed_frame_pose_in_map.rotation()),
                        Eigen::Vector3d::UnitZ())),
                nullptr,
                absl::make_unique<ceres::AutoDiffLocalParameterization<
                    YawOnlyQuaternionPlus, 4, 1>>(),
                &problem));
        fixed_frame_pose_initialized = true;
      }

      problem.AddResidualBlock(
          SpaCostFunction3D::CreateAutoDiffCostFunction(constraint_pose),
          options_.fixed_frame_pose_use_tolerant_loss() ?
              new ceres::TolerantLoss(
            options_.fixed_frame_pose_tolerant_loss_param_a(),
            options_.fixed_frame_pose_tolerant_loss_param_b()) : nullptr,
          C_fixed_frames.at(trajectory_id).rotation(),
          C_fixed_frames.at(trajectory_id).translation(),
          C_nodes.at(node_id).rotation(), C_nodes.at(node_id).translation());
    }
  }
  // Solve.
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
    for (const auto& trajectory_id_and_data : trajectory_data_) {
      const int trajectory_id = trajectory_id_and_data.first;
      const TrajectoryData& trajectory_data = trajectory_id_and_data.second;
      if (trajectory_id != 0) {
        LOG(INFO) << "Trajectory " << trajectory_id << ":";
      }
      LOG(INFO) << "Gravity was: " << trajectory_data.gravity_constant;
      const auto& imu_calibration = trajectory_data.imu_calibration;
      LOG(INFO) << "IMU correction was: "
                << common::RadToDeg(2. *
                                    std::acos(std::abs(imu_calibration[0])))
                << " deg (" << imu_calibration[0] << ", " << imu_calibration[1]
                << ", " << imu_calibration[2] << ", " << imu_calibration[3]
                << ")";
    }
  }

  // Store the result.
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        C_submap_id_data.data.ToRigid();
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose =
        C_node_id_data.data.ToRigid();
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        C_fixed_frame.second.ToRigid();
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

std::unique_ptr<transform::Rigid3d>
OptimizationProblem3D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec3D& first_node_data,
    const NodeSpec3D& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        Interpolate(odometry_data_, trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        Interpolate(odometry_data_, trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      const transform::Rigid3d relative_odometry =
          first_node_odometry->inverse() * (*second_node_odometry);
      return absl::make_unique<transform::Rigid3d>(relative_odometry);
    }
  }
  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
```