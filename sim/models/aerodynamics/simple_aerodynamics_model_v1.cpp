#include "sim/models/aerodynamics/simple_aerodynamics_model_v1.hpp"

#include <cmath>

namespace sixsim::sim {

namespace {

constexpr double pi() {
  return 3.141592653589793238462643383279502884;
}

math::Vector3 drag_force_body(const AerodynamicState& aerodynamic_state,
                              double reference_area_m2,
                              double drag_coefficient) {
  if (aerodynamic_state.airspeed_mps <= 0.0) {
    return {};
  }

  const double drag_n = aerodynamic_state.dynamic_pressure_pa *
                        reference_area_m2 * drag_coefficient;
  return aerodynamic_state.air_relative_velocity_body_mps *
         (-drag_n / aerodynamic_state.airspeed_mps);
}

math::Vector3 fin_normal_force_body(
    const AerodynamicState& aerodynamic_state,
    const SimpleAerodynamicsModelV1Params& params) {
  math::Vector3 force_body_n{};

  if (params.fin_count <= 0) {
    return force_body_n;
  }

  for (int fin_index = 0; fin_index < params.fin_count; ++fin_index) {
    const double psi_rad =
        2.0 * pi() * static_cast<double>(fin_index) /
        static_cast<double>(params.fin_count);
    const double alpha_i_rad =
        aerodynamic_state.angle_of_attack_rad * std::cos(psi_rad) -
        aerodynamic_state.sideslip_rad * std::sin(psi_rad);
    const double normal_force_n =
        aerodynamic_state.dynamic_pressure_pa * params.fin_planform_area_m2 *
        params.fin_normal_force_slope_per_rad * alpha_i_rad;
    const math::Vector3 normal_direction_body{
        0.0,
        -std::sin(psi_rad),
        std::cos(psi_rad),
    };

    force_body_n -= normal_direction_body * normal_force_n;
  }

  return force_body_n;
}

math::Vector3 nose_normal_force_body(
    const AerodynamicState& aerodynamic_state,
    double fin_planform_area_m2,
    double nose_normal_force_slope_per_rad) {
  const math::Vector3& velocity_body =
      aerodynamic_state.air_relative_velocity_body_mps;
  const double off_axis_speed_mps =
      std::hypot(velocity_body.y, velocity_body.z);

  if (aerodynamic_state.airspeed_mps <= 0.0 || off_axis_speed_mps <= 0.0) {
    return {};
  }

  const double alpha_eff_rad =
      std::atan2(off_axis_speed_mps, velocity_body.x);
  const double normal_force_n = aerodynamic_state.dynamic_pressure_pa *
                                fin_planform_area_m2 *
                                nose_normal_force_slope_per_rad *
                                alpha_eff_rad;
  const math::Vector3 off_axis_direction_body{
      0.0,
      velocity_body.y / off_axis_speed_mps,
      velocity_body.z / off_axis_speed_mps,
  };

  return off_axis_direction_body * -normal_force_n;
}

}  // namespace

SimpleAerodynamicsModelV1::SimpleAerodynamicsModelV1(
    const SimpleAerodynamicsModelV1Params& params)
    : params_(params) {}

ForceMoment SimpleAerodynamicsModelV1::evaluate(
    const RigidBodyState& state,
    const AerodynamicState& aerodynamic_state,
    const VehicleContext& vehicle) const {
  (void)state;
  (void)vehicle;

  ForceMoment force_moment{};
  force_moment.force_body_n +=
      drag_force_body(aerodynamic_state,
                      params_.reference_area_m2,
                      params_.drag_coefficient);
  force_moment.force_body_n +=
      fin_normal_force_body(aerodynamic_state, params_);
  force_moment.force_body_n +=
      nose_normal_force_body(aerodynamic_state,
                             params_.fin_planform_area_m2,
                             params_.nose_normal_force_slope_per_rad);
  return force_moment;
}

}  // namespace sixsim::sim
