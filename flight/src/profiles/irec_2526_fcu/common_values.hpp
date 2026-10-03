#pragma once

namespace sixsim::flight::irec_2526_fcu {

struct CommonValues {
  double altitude_msl_m_f64{};
  bool altitude_valid_bool{false};
  double doubled_altitude_msl_m_f64{};
  double tripled_altitude_msl_m_f64{};

  // Transition flags
  bool enter_powered_ascent{false};
  bool enter_coasting_ascent{false};
  bool enter_drogue_descent{false};
  bool enter_main_descent{false};
  bool enter_landed{false};
};

}  // namespace sixsim::flight::irec_2526_fcu
