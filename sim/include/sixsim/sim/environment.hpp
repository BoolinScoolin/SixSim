#pragma once

#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/gravity.hpp"
#include "sixsim/sim/models/wind.hpp"
#include "sixsim/sim/sim_time.hpp"

namespace sixsim::sim {

struct EnvironmentState {
  AtmosphereState atmosphere{};
  WindState wind{};
  GravityState gravity{};
};

inline EnvironmentState evaluate_environment(
    const SimTime& time,
    const math::Vector3& position_ned_m,
    const AtmosphereModel& atmosphere_model,
    const WindModel& wind_model,
    const GravityModel& gravity_model) {
  EnvironmentState environment{};
  environment.atmosphere = atmosphere_model.evaluate(time, position_ned_m);
  environment.wind = wind_model.evaluate(time, position_ned_m);
  environment.gravity = gravity_model.evaluate(time, position_ned_m);
  return environment;
}

}  // namespace sixsim::sim
