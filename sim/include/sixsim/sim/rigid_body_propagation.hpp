#pragma once

#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/rigid_body_rk4.hpp"
#include "sixsim/sim/rigid_body_state.hpp"

namespace sixsim::sim {

inline RigidBodyState propagate_rigid_body(
    const RigidBodyState& state,
    const ForceMoment& force_moment,
    const MassProperties& mass_properties,
    double dt_s) {
  return integrate_rigid_body_rk4(state, force_moment, mass_properties, dt_s);
}

}  // namespace sixsim::sim
