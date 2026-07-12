#include "constant_atmosphere.hpp"

namespace sixsim::sim {

ConstantAtmosphere::ConstantAtmosphere(const AtmosphereState& atmosphere)
    : atmosphere_(atmosphere) {}

AtmosphereState ConstantAtmosphere::evaluate(
    const SimTime& time,
    const math::Vector3& position_ned_m) const {
  (void)time;
  (void)position_ned_m;
  return atmosphere_;
}

}  // namespace sixsim::sim
