#include "sixsim/sim/run_simulation.hpp"

#include "sixsim/sim/logging.hpp"
#include "sixsim/sim/run_artifacts.hpp"

#include "scenario_config.hpp"
#include "sim/models/dynamics/rigid_body.hpp"

#include <iomanip>
#include <iostream>
#include <utility>

namespace sixsim::sim {

Simulation::Simulation(std::filesystem::path output_directory)
    : output_directory_(std::move(output_directory)),
      scenario_(build_scenario()) {}

RigidBodyState Simulation::update_vehicle_state(
    const SimTime& time,
    const RigidBodyState& state,
    double dt_s,
    const AuxiliaryContext& auxiliary) {
  const AtmosphereState atmosphere =
      auxiliary.atmosphere_model.evaluate(time, state.position_ned_m);
  const WindState wind =
      auxiliary.wind_model.evaluate(time, state.position_ned_m);
  const GravityState gravity =
      auxiliary.gravity_model.evaluate(time, state.position_ned_m);
  const AerodynamicState aerodynamic_state =
      compute_aerodynamic_state(state, atmosphere, wind);

  const ForceMoment propulsion_force_moment =
      auxiliary.propulsion_model.evaluate(time, state, auxiliary.vehicle);
  const ForceMoment gravity_force_moment =
      gravity_force_moment_body(gravity,
                                auxiliary.vehicle.mass_properties,
                                state.q_body2ned);
  const ForceMoment aerodynamics_force_moment =
      auxiliary.aerodynamics_model.evaluate(state,
                                            aerodynamic_state,
                                            auxiliary.vehicle);

  const ForceMoment force_moment =
      combine_force_moment(gravity_force_moment,
                           aerodynamics_force_moment,
                           propulsion_force_moment);

  return advance_state(
      state,
      dt_s,
      [&](const auto& integration_state) {
        return rigid_body_derivative(integration_state,
                                     force_moment,
                                     auxiliary.vehicle.mass_properties);
      },
      post_step_rigid_body);
}

void Simulation::run() {
  const RunArtifacts artifacts{
      output_directory_,
      scenario_.default_run_directory,
      scenario_.source_scenario_path,
  };
  SimulationLogger log{artifacts.run_directory(), scenario_.logging_rate_hz};

  VehicleContext vehicle_context{
      scenario_.vehicle.mass_properties,
      scenario_.vehicle.unloaded_mass_kg,
      scenario_.vehicle.actuator,
  };

  const AuxiliaryContext auxiliary{
      vehicle_context,
      *scenario_.environment.atmosphere,
      *scenario_.environment.wind,
      *scenario_.environment.gravity,
      *scenario_.vehicle.aerodynamics,
      *scenario_.vehicle.propulsion,
  };

  while (!events_.stop_simulation.triggered) {
    evaluate_simulation_events(
        time_, scenario_.vehicle.state, scenario_.simulation, events_);
    if (events_.stop_simulation.triggered) {
      break;
    }

    log.log_truth(time_, scenario_.vehicle.state);
    scenario_.vehicle.state = update_vehicle_state(
        time_,
        scenario_.vehicle.state,
        scenario_.simulation.dt_s,
        auxiliary);
    time_.simtime_s += scenario_.simulation.dt_s;
  }

  artifacts.save_manifest();

  std::cout << "final simtime: " << time_.simtime_s << '\n';
  std::cout << "stop simulation trigger time: "
            << events_.stop_simulation.trigger_time_s << '\n';
  std::cout << "final position NED z: " << std::fixed << std::setprecision(9)
            << scenario_.vehicle.state.position_ned_m.z << '\n';
  std::cout << "final velocity body x: " << std::fixed << std::setprecision(9)
            << scenario_.vehicle.state.velocity_body_mps.x << '\n';
  std::cout << "final velocity body y: " << std::fixed << std::setprecision(9)
            << scenario_.vehicle.state.velocity_body_mps.y << '\n';
  std::cout << "final mass: " << std::fixed << std::setprecision(9)
            << scenario_.vehicle.mass_properties.mass_kg << '\n';
}

}  // namespace sixsim::sim
