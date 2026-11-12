#pragma once

#include "sixsim/math.hpp"

namespace sixsim::flight::irec_2526_fcu {

struct CommonValues {
  /*
   * Hardware Outputs
   */

  // Altimeter
  double altimeter_altitude_msl_m_f64{};
  bool altitude_valid_bool{false};

  // IMU
  Vector3 imu_accel_mps2_v3{};
  bool imu_accel_valid_bool{false};
  Vector3 imu_gyro_rps_v3{};
  bool imu_gyro_valid_bool{false};

  /*
   * Algorithm Variables
   */
  double doubled_altitude_msl_m_f64{};
  double tripled_altitude_msl_m_f64{};

  /*
   * Transition Flags
   */
  bool enter_powered_ascent{false};
  bool enter_coasting_ascent{false};
  bool enter_drogue_descent{false};
  bool enter_main_descent{false};
  bool enter_landed{false};
};

}  // namespace sixsim::flight::irec_2526_fcu
