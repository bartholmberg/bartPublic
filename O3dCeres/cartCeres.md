

# Ceres in Cartographer

See how Ceres uses constraints; for instance 3d Pose and gravity vector 
##     *imu_based_pose_extrapolator.cc*

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

[optimization_problem_3d][cartographer/optimization_problem_3d.cc at master · cartographer-project/cartographer (github.com)](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/optimization/optimization_problem_3d.cc)

```c++ start=435
...
}

      // Force gravity constant to be positive.
      problem.SetParameterLowerBound(&trajectory_data.gravity_constant, 0, 0.0);
    }
  }

```



* test
* test2



