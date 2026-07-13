#pragma once

#include "sixsim/math/quaternion.hpp"
#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/models/gravity.hpp"

namespace sixsim::sim {

ForceMoment gravity_force_moment_body(const GravityState& gravity,
                                      const MassProperties& mass_properties,
                                      const math::Quaternion& q_body2ned);

}  // namespace sixsim::sim
