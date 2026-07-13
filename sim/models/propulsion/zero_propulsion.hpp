#pragma once

#include "sixsim/sim/models/propulsion.hpp"

namespace sixsim::sim {

class ZeroPropulsion final : public PropulsionModel {
 public:
  ForceMoment evaluate(const SimTime& time,
                       const RigidBodyState& state,
                       const VehicleContext& vehicle) const override;
};

}  // namespace sixsim::sim
