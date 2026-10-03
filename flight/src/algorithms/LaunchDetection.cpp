#include "flight/build/generated/include/sixsim/flight/algorithms/LaunchDetection.hpp"

namespace sixsim::flight::algorithms {

void LaunchDetection::enter() {}

void LaunchDetection::run() {
  outputs.enter_powered_ascent = inputs.altitude_msl_m_f64 > 50.0;
}

void LaunchDetection::exit() {}

}  // namespace sixsim::flight::algorithms
