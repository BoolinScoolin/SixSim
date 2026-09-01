#pragma once

#include "sixsim/sim/models/aerodynamics.hpp"

namespace sixsim::sim {

struct SimpleAerodynamicsModelV1Params {
  double reference_area_m2{};
  double drag_coefficient{};
  int fin_count{};
  double fin_planform_area_m2{};
  double fin_normal_force_slope_per_rad{};
  double nose_normal_force_slope_per_rad{};
};

class SimpleAerodynamicsModelV1 final : public AerodynamicsModel {
 public:
  explicit SimpleAerodynamicsModelV1(
      const SimpleAerodynamicsModelV1Params& params);

  ForceMoment evaluate(const RigidBodyState& state,
                       const AerodynamicState& aerodynamic_state,
                       const VehicleContext& vehicle) const override;

 private:
  SimpleAerodynamicsModelV1Params params_{};
};

}  // namespace sixsim::sim
