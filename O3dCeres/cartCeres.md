## *Point Cloud Registration- Open3D and Cartographer*

Open3d and Cartographer use external solvers for registering point clouds.  Open3d currently uses eigen, and Cartographer uses Ceres.  This note compares use of the two and potential use of Ceres in Open3d.

## Ceres in Cartographer 

See how Ceres uses constraints in Cartographer; for instance 3d Pose and gravity vector 
###     *imu_based_pose_extrapolator.cc*

[imu_based_pose_extrapolator.cc]https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/imu_based_pose_extrapolator.cc
```c++ start=272
...
if (fix_gravity) {
        problem.SetParameterBlockConstant(&gravity_constant);
      } else {
        // Force gravity constant to be positive.
        problem.SetParameterLowerBound(&gravity_constant, 0, 0.0);
      }
    }
```
### optimization_problem_3d
[optimization_problem_3d][cartographer/optimization_problem_3d.cc at master · cartographer-project/cartographer (github.com)](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc)

```c++ start=435
...
}

      // Force gravity constant to be positive.
      problem.SetParameterLowerBound(&trajectory_data.gravity_constant, 0, 0.0);
    }
  }

```
## Cartographer constraints

### inter submaps constraints


*Non-global constraints*  (also known as  inter submaps{: .blue} . constraints) are built automatically between nodes that are closely following each other on a trajectory. Intuitively, those “non-global ropes” keep the local structure of the trajectory coherent.

Global constraints (also referred to as loop closure constraints or intra submaps contraints) are regularly searched between a new submap and previous nodes that are considered “close enough” in space (part of a certain search window) and a strong fit (a good match when running scan matching). Intuitively, those “global ropes” introduce knots in the structure and firmly bring two strands closer."

### intra submaps constraints

A constraint represents the estimated pose of the *node j relative to the submap i* . They can be differentiated between intra-submap (where node j was inserted into submap i) and inter-submap constraints (where node j  was NOT  inserted into submap i).

Basically, the intra constraints compose the "backbone" of the trajectory, while inter constraints represents loop closings in one trajectory or the joining of different trajectories (including the fixed one that should be supplied when running in pure localization mode).
* test
* test 
* 
## Open3D Eigen solver



