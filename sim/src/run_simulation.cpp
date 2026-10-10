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
      scenario_(build_scenario()) {
  vehicles_.push_back(build_vehicle());
}

ForceMoment Simulation::evaluate_vehicle_force_moment(
    ConfiguredVehicle& vehicle,
    const RigidBodyState& state,
    const SimTime& time) {
  VehicleContext vehicle_context{
      vehicle.mass_properties,
      vehicle.unloaded_mass_kg,
      vehicle.actuator,
  };

  const AuxiliaryContext auxiliary{
      vehicle_context,
      *scenario_.environment.atmosphere,
      *scenario_.environment.wind,
      *scenario_.environment.gravity,
      *vehicle.aerodynamics,
      *vehicle.propulsion,
  };

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
  const ForceMoment gravity_force_moment = gravity_force_moment_body(
      gravity, auxiliary.vehicle.mass_properties, state.q_body2ned);
  const ForceMoment aerodynamics_force_moment =
      auxiliary.aerodynamics_model.evaluate(state,
                                            aerodynamic_state,
                                            auxiliary.vehicle);
  return combine_force_moment(gravity_force_moment,
                              aerodynamics_force_moment,
                              propulsion_force_moment);
}

void Simulation::update_vehicle_state(ConfiguredVehicle& vehicle) {
  if (!vehicle.dynamics_enabled) {
    vehicle.derivative = {};
    return;
  }

  const RigidBodyState& state = vehicle.state;
  const auto evaluate_derivative =
      [&](const auto& integration_state, double time_offset_s) {
        const SimTime stage_time{time_.simtime_s + time_offset_s};
        const ForceMoment force_moment = evaluate_vehicle_force_moment(
            vehicle, integration_state, stage_time);
        return rigid_body_derivative(
            integration_state, force_moment, vehicle.mass_properties);
      };

  vehicle.state = advance_state(state,
                                scenario_.simulation.dt_s,
                                evaluate_derivative,
                                post_step_rigid_body);
  vehicle.derivative =
      evaluate_derivative(vehicle.state, scenario_.simulation.dt_s);
}

void Simulation::run() {
  ConfiguredVehicle& vehicle = vehicles_.front();
  const RunArtifacts artifacts{
      output_directory_,
      scenario_.default_run_directory,
      scenario_.source_scenario_path,
      scenario_.fcu_config_paths,
  };
  SimulationLogger log{artifacts.run_directory(), scenario_.logging_rate_hz};

  while (true) {
    // Update simulation events based on the current simulation state.
    evaluate_simulation_events(
        time_, vehicles_, scenario_.simulation, events_);
    if (events_.stop_simulation.triggered) {
      break;
    }

    // Log current state.
    log.log_truth(time_, vehicle.state);

    /*
     * Advance vehicle state. Updates vehicle state to live at time t_k+1
     *
     * Vehicle derivative at time t_k+1 is also estimated here,
     * but it is computed using the achieved control input u_k.
     * This is necessary because updating u_k to u_k+1 requires
     * measurements at time t_k+1, so we estimate the derivative
     * to drive sensor models.
     */
    for (ConfiguredVehicle& current_vehicle : vehicles_) {
      update_vehicle_state(current_vehicle);
    }

    // Advance simulation time. Updates time to t_k+1
    time_.simtime_s += scenario_.simulation.dt_s;
    for (ConfiguredVehicle& current_vehicle : vehicles_) {

      /*
       * Update FCU. This updates sensors and FSW.
       * Measurements now lives at t_k+1
       * Control command now lives at t_k+1
       */
      current_vehicle.update_fcus(time_, scenario_);

      /*
       * NOT IMPLEMENTED: Update actuators. Actuator state now lives at t_k+1.
       */

      /*
       * Recompute derivative using updated actuator state. Vehicle state derivative now lives at t_k+1.
       */
      if (!current_vehicle.dynamics_enabled) {
        current_vehicle.derivative = {};
        continue;
      }
      const ForceMoment force_moment = evaluate_vehicle_force_moment(
          current_vehicle, current_vehicle.state, time_);
      current_vehicle.derivative = rigid_body_derivative(
          current_vehicle.state, force_moment, current_vehicle.mass_properties);
    }
  }

  artifacts.save_manifest();

  std::cout << "\n===== SIMULATION FINISHED ===== " << '\n';
  std::cout << "final simtime: " << time_.simtime_s << '\n';
  std::cout << "stop simulation trigger time: "
            << events_.stop_simulation.trigger_time_s << '\n';
  std::cout << "final position NED z: " << std::fixed << std::setprecision(9)
            << vehicle.state.position_ned_m.z << '\n';
  std::cout << "final velocity body x: " << std::fixed << std::setprecision(9)
            << vehicle.state.velocity_body_mps.x << '\n';
  std::cout << "final velocity body y: " << std::fixed << std::setprecision(9)
            << vehicle.state.velocity_body_mps.y << '\n';
  std::cout << "final mass: " << std::fixed << std::setprecision(9)
            << vehicle.mass_properties.mass_kg << '\n';
}

}  // namespace sixsim::sim
