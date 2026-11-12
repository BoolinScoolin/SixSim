#include "flight/build/generated/include/sixsim/flight/algorithms/LaunchDetection.hpp"

namespace sixsim::flight::algorithms {

void LaunchDetection::enter() {}

void LaunchDetection::run() {
  double accel_norm_mps2 = math::norm(inputs.imu_accel_mps2_v3);
  outputs.enter_powered_ascent =
      inputs.imu_accel_valid_bool && accel_norm_mps2 > 20;
}

void LaunchDetection::exit() {}

}  // namespace sixsim::flight::algorithms
