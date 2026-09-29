#pragma once

namespace sixsim::flight::irec_2526_fcu {

struct CommonValues {
  double altitude_msl_m_f64{};
  bool altitude_valid_bool{false};
  double doubled_altitude_msl_m_f64{};
  double tripled_altitude_msl_m_f64{};
};

}  // namespace sixsim::flight::irec_2526_fcu
