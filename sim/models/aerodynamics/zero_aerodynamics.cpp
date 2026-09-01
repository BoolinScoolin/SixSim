#include "sim/models/aerodynamics/zero_aerodynamics.hpp"

namespace sixsim::sim {

ForceMoment ZeroAerodynamics::evaluate(const RigidBodyState& state,
                                       const AerodynamicState& aerodynamic_state,
                                       const VehicleContext& vehicle) const {
  (void)state;
  (void)aerodynamic_state;
  (void)vehicle;
  return {};
}

}  // namespace sixsim::sim
