#include "sim/models/aerodynamics/zero_aerodynamics.hpp"

namespace sixsim::sim {

ForceMoment ZeroAerodynamics::evaluate(const RigidBodyState& state,
                                       const AtmosphereState& atmosphere,
                                       const WindState& wind,
                                       const VehicleContext& vehicle) const {
  (void)state;
  (void)atmosphere;
  (void)wind;
  (void)vehicle;
  return {};
}

}  // namespace sixsim::sim
