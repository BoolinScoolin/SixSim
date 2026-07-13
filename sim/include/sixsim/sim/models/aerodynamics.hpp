#pragma once

#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/wind.hpp"
#include "sixsim/sim/rigid_body_state.hpp"
#include "sixsim/sim/vehicle_context.hpp"

namespace sixsim::sim {

class AerodynamicsModel {
 public:
  virtual ~AerodynamicsModel() = default;

  virtual ForceMoment evaluate(const RigidBodyState& state,
                               const AtmosphereState& atmosphere,
                               const WindState& wind,
                               const VehicleContext& vehicle) const = 0;
};

}  // namespace sixsim::sim
