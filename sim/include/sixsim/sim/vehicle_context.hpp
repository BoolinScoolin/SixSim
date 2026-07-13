#pragma once

#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/models/actuator.hpp"

namespace sixsim::sim {

struct VehicleContext {
  const MassProperties& mass_properties;
  const ActuatorState& actuator;
};

}  // namespace sixsim::sim
