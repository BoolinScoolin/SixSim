#pragma once

#include "sixsim/math.hpp"

namespace sixsim::hal {

struct IMU6DOFSample {
  Vector3 a_accel_imu_mps2{};
  Vector3 w_gyro_imu_rps{};
  double measurement_time_s{};  // Measurement time since runtime start.
  bool valid{false};
};

class imu_6dof {
 public:
  virtual ~imu_6dof() = default;

  // Return the latest sample without waiting for a new measurement.
  // Until a measurement is available, return a sample with valid == false.
  virtual IMU6DOFSample read() = 0;
};

}  // namespace sixsim::hal
