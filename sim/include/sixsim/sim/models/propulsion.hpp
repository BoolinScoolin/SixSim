#pragma once

#include "sixsim/sim/sim_general.hpp"

#include "sim/models/dynamics/rigid_body.hpp"

namespace sixsim::sim {

class PropulsionModel {
 public:
  virtual ~PropulsionModel() = default;

  virtual ForceMoment evaluate(const SimTime& time,
                               const RigidBodyState& state,
                               const VehicleContext& vehicle) const = 0;
};

}  // namespace sixsim::sim
