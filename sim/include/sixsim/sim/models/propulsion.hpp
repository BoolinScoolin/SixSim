#pragma once

#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/rigid_body_state.hpp"
#include "sixsim/sim/sim_time.hpp"
#include "sixsim/sim/vehicle_context.hpp"

namespace sixsim::sim {

class PropulsionModel {
 public:
  virtual ~PropulsionModel() = default;

  virtual ForceMoment evaluate(const SimTime& time,
                               const RigidBodyState& state,
                               const VehicleContext& vehicle) const = 0;
};

}  // namespace sixsim::sim
