#include "sim/models/propulsion/zero_propulsion.hpp"

namespace sixsim::sim {

ForceMoment ZeroPropulsion::evaluate(const SimTime& time,
                                     const RigidBodyState& state,
                                     const VehicleContext& vehicle) const {
  (void)time;
  (void)state;
  (void)vehicle;
  return {};
}

}  // namespace sixsim::sim
