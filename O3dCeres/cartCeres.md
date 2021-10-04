

# Ceres in Cartographer

See how Ceres uses constraints; for instance 3d Pose and gravity vector 
##     *imu_based_pose_extrapolator.cc*

[imu_based_pose_extrapolator.cc]https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/imu_based_pose_extrapolator.cc
```c++
...
if (fix_gravity) {
        problem.SetParameterBlockConstant(&gravity_constant);
      } else {
        // Force gravity constant to be positive.
        problem.SetParameterLowerBound(&gravity_constant, 0, 0.0);
      }
    }
```
**Core features of Open3D include:**

* test
* test2



