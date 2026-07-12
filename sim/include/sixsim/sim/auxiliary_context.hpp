#pragma once

#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/models/actuator.hpp"
#include "sixsim/sim/models/aerodynamics.hpp"
#include "sixsim/sim/models/atmosphere.hpp"
#include "sixsim/sim/models/gravity.hpp"
#include "sixsim/sim/models/propulsion.hpp"
#include "sixsim/sim/models/wind.hpp"

namespace sixsim::sim {

struct AuxiliaryContext {
  const ActuatorState& actuator;
  const MassProperties& mass_properties;
  const AtmosphereModel& atmosphere_model;
  const WindModel& wind_model;
  const GravityModel& gravity_model;
  const AerodynamicsModel& aerodynamics_model;
  const PropulsionModel& propulsion_model;
};

}  // namespace sixsim::sim
