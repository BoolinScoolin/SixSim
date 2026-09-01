#pragma once

#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/models/propulsion.hpp"

namespace sixsim::sim {

class HardcodedThrustCurvePropulsion final : public PropulsionModel {
 public:
  explicit HardcodedThrustCurvePropulsion(
      const math::Vector3& thrust_direction_body);

  ForceMoment evaluate(const SimTime& time,
                       const RigidBodyState& state,
                       VehicleContext& vehicle) const override;

 private:
  math::Vector3 thrust_direction_body_{};
};

}  // namespace sixsim::sim
