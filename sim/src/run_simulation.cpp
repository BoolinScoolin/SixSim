#include "sixsim/sim/run_simulation.hpp"

#include "sixsim/sim/auxiliary_context.hpp"
#include "sixsim/sim/force_moment.hpp"
#include "sixsim/sim/mass_properties.hpp"
#include "sixsim/sim/rigid_body_propagation.hpp"
#include "sixsim/sim/rigid_body_state.hpp"
#include "sixsim/sim/simulation_config.hpp"
#include "sixsim/sim/simulation_event.hpp"
#include "sixsim/sim/sim_time.hpp"
#include "sixsim/sim/vehicle_context.hpp"

#include "sim/models/aerodynamics/zero_aerodynamics.hpp"
#include "sim/models/atmosphere/constant_atmosphere.hpp"
#include "sim/models/dynamics/rigid_body_dynamics.hpp"
#include "sim/models/gravity/constant_gravity.hpp"
#include "sim/models/propulsion/zero_propulsion.hpp"
#include "sim/models/wind/constant_wind.hpp"

#include <iostream>

namespace sixsim::sim {

void run_simulation() {
  SimulationConfig config{};
  config.dt_s = 0.001;
  config.stop_simulation_time_s = 1.0;

  SimulationEvents events{};
  SimTime time{};
  RigidBodyState state{};
  state.position_ned_m = {0.0, 0.0, 0.0};
  state.velocity_body_mps = {0.0, 0.0, 0.0};
  state.omega_body_rps = {0.0, 0.0, 0.0};

  AtmosphereState atmosphere_state{};
  atmosphere_state.density_kg_per_m3 = 1.225;
  atmosphere_state.pressure_pa = 101325.0;
  atmosphere_state.temperature_k = 288.15;
  atmosphere_state.speed_of_sound_m_per_s = 340.3;
  const ConstantAtmosphere atmosphere_model{atmosphere_state};

  const math::Vector3 wind_ned_mps{0.0, 0.0, 0.0};
  const ConstantWind wind_model{wind_ned_mps};

  const double gravity_mps2 = 9.80665;
  const ConstantGravity gravity_model{gravity_mps2};

  const ZeroAerodynamics aerodynamics_model{};
  const ZeroPropulsion propulsion_model{};

  const ActuatorState actuator{};

  MassProperties mass_properties{};
  mass_properties.mass_kg = 10.0;
  mass_properties.inertia_body_kgm2 = {1.0, 1.0, 1.0};

  const VehicleContext vehicle{
      mass_properties,
      actuator,
  };

  const AuxiliaryContext auxiliary{
      vehicle,
      atmosphere_model,
      wind_model,
      gravity_model,
      aerodynamics_model,
      propulsion_model,
  };

  while (!events.stop_simulation.triggered) {
    evaluate_simulation_events(time, state, config, events);
    if (events.stop_simulation.triggered) {
      break;
    }

    const AtmosphereState atmosphere =
        auxiliary.atmosphere_model.evaluate(time, state.position_ned_m);
    const WindState wind =
        auxiliary.wind_model.evaluate(time, state.position_ned_m);
    const GravityState gravity =
        auxiliary.gravity_model.evaluate(time, state.position_ned_m);

    const ForceMoment gravity_force_moment =
        gravity_force_moment_body(gravity, auxiliary.vehicle.mass_properties,
                                  state.q_body2ned);
    const ForceMoment aerodynamics_force_moment =
        auxiliary.aerodynamics_model.evaluate(state, atmosphere, wind,
                                              auxiliary.vehicle);
    const ForceMoment propulsion_force_moment =
        auxiliary.propulsion_model.evaluate(time, state, auxiliary.vehicle);
    const ForceMoment force_moment =
        combine_force_moment(gravity_force_moment, aerodynamics_force_moment,
                             propulsion_force_moment);

    state = propagate_rigid_body(state, force_moment,
                                 auxiliary.vehicle.mass_properties,
                                 config.dt_s);
    time.simtime_s += config.dt_s;
  }

  std::cout << "final simtime: " << time.simtime_s << '\n';
  std::cout << "stop simulation trigger time: "
            << events.stop_simulation.trigger_time_s << '\n';
  std::cout << "final position NED z: " << state.position_ned_m.z << '\n';
}

}  // namespace sixsim::sim
