#include "flight/build/generated/include/sixsim/flight/algorithms/DoubleAltitude.hpp"

namespace sixsim::flight::algorithms {

void DoubleAltitude::enter() {}

void DoubleAltitude::run() {
  outputs.doubled_altitude_msl_m_f64 = 2.0 * inputs.altitude_msl_m_f64;
}

void DoubleAltitude::exit() {}

}  // namespace sixsim::flight::algorithms
