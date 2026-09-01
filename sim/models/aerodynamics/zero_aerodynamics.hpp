#pragma once

#include "sixsim/sim/models/aerodynamics.hpp"

namespace sixsim::sim {

class ZeroAerodynamics final : public AerodynamicsModel {
 public:
  ForceMoment evaluate(const RigidBodyState& state,
                       const AerodynamicState& aerodynamic_state,
                       const VehicleContext& vehicle) const override;
};

}  // namespace sixsim::sim
