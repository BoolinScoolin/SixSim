#include "flight/build/generated/include/sixsim/flight/algorithms/TripleAltitude.hpp"

namespace sixsim::flight::algorithms {

void TripleAltitude::enter() {}

void TripleAltitude::run() {
  outputs.tripled_altitude_msl_m_f64 = 3.0 * inputs.altitude_msl_m_f64;
}

void TripleAltitude::exit() {}

}  // namespace sixsim::flight::algorithms
