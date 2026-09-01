#!/usr/bin/env python3

from pathlib import Path
import argparse
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_PYTHON_DIR = REPO_ROOT / "tools" / "python"
sys.path.insert(0, str(TOOLS_PYTHON_DIR))

try:
    import yaml
except ModuleNotFoundError:
    print(
        "Missing dependency: PyYAML. Install with "
        "`python3 -m pip install -r tools/codegen/requirements.txt`.",
        file=sys.stderr,
    )
    raise SystemExit(1)

try:
    from sixsim_gnc import q_body2ned_from_az_el_roll
except ModuleNotFoundError:
    print(
        f"Missing local package: sixsim_gnc. Expected it under {TOOLS_PYTHON_DIR}.",
        file=sys.stderr,
    )
    raise SystemExit(1)


# Generate run_simulation.cpp manually before configuring or building:
#
#   python -m pip install -r tools/codegen/requirements.txt
#   python tools/codegen/generate_run_simulation.py configs/scenarios/test/smoke.yaml build/generated/run_simulation.cpp
#
# Generated C++ lives under build/ and is not committed.
#
# To register a new model or integrator type, add its parser, C++ include
# renderer, C++ construction or advance-state renderer, and table entry in that
# category's section below. Register entries immediately below their functions.


def require_mapping(value, path):
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def require_number(mapping, key, path):
    if key not in mapping:
        raise ValueError(f"{path}.{key} is required")
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path}.{key} must be a number")
    return float(value)


def require_int(mapping, key, path):
    if key not in mapping:
        raise ValueError(f"{path}.{key} is required")
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path}.{key} must be an integer")
    return value


def require_type(mapping, valid_types, path):
    actual = mapping.get("type")
    if actual not in valid_types:
        valid = ", ".join(sorted(repr(value) for value in valid_types))
        raise ValueError(f"{path}.type must be one of {valid}, got {actual!r}")
    return actual


def require_value(mapping, key, valid_values, path):
    actual = mapping.get(key)
    if actual not in valid_values:
        valid = ", ".join(sorted(repr(value) for value in valid_values))
        raise ValueError(f"{path}.{key} must be one of {valid}, got {actual!r}")
    return actual


def require_vector(mapping, key, length, path):
    if key not in mapping:
        raise ValueError(f"{path}.{key} is required")
    value = mapping[key]
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{path}.{key} must be a list of {length} numbers")
    if not all(
        not isinstance(item, bool) and isinstance(item, (int, float))
        for item in value
    ):
        raise ValueError(f"{path}.{key} must be a list of {length} numbers")
    return [float(item) for item in value]


def cpp_number(value):
    return repr(float(value))


def cpp_vector(values):
    return "{" + ", ".join(cpp_number(value) for value in values) + "}"


def cpp_string(value):
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def model(type_name, parse, include, construct):
    return {
        "type": type_name,
        "parse": parse,
        "include": include,
        "construct": construct,
    }


def integrator(type_name, parse, include, advance_state):
    return {
        "type": type_name,
        "parse": parse,
        "include": include,
        "advance_state": advance_state,
    }


def parse_model(category, model_config, support_table):
    path = f"models.{category}"
    model_type = require_type(model_config, support_table.keys(), path)
    model_support = support_table[model_type]
    return {
        "type": model_type,
        "data": model_support["parse"](model_config, path),
        "include": model_support["include"],
        "construct": model_support["construct"],
    }


def render_model_include(parsed_model):
    return parsed_model["include"](parsed_model["data"])


def render_model_construction(parsed_model):
    return parsed_model["construct"](parsed_model["data"])


def parse_initial_attitude(rigid_body):
    path = "initial_state.rigid_body"
    has_q_body2ned = "q_body2ned" in rigid_body
    aer_fields = ("azimuth_rad", "elevation_rad", "roll_rad")
    present_aer_fields = [field for field in aer_fields if field in rigid_body]
    has_aer = bool(present_aer_fields)

    if has_q_body2ned and has_aer:
        raise ValueError(
            f"{path} must specify either q_body2ned or "
            "azimuth_rad/elevation_rad/roll_rad, not both"
        )

    if has_q_body2ned:
        return require_vector(rigid_body, "q_body2ned", 4, path)

    if has_aer:
        missing_aer_fields = [
            field for field in aer_fields if field not in rigid_body
        ]
        if missing_aer_fields:
            missing = ", ".join(missing_aer_fields)
            raise ValueError(f"{path} is missing AER field(s): {missing}")
        return q_body2ned_from_az_el_roll(
            require_number(rigid_body, "azimuth_rad", path),
            require_number(rigid_body, "elevation_rad", path),
            require_number(rigid_body, "roll_rad", path),
        )

    raise ValueError(
        f"{path} must specify q_body2ned or azimuth_rad/elevation_rad/roll_rad"
    )


# =============================================================================
# Integrators
# =============================================================================

INTEGRATORS = {}

# RK4

def parse_rk4_integrator(_simulation_config, _path):
    return {}


def include_rk4_integrator(_parsed):
    return '#include "sixsim/sim/rk4.hpp"'


def construct_rk4_integrator(_parsed):
    return """  const auto advance_state_rk4 =
      [](const auto& current_state,
         double dt_s,
         const auto& derivative_function,
         const auto& post_step_routine) {
        return step_rk4(
            current_state,
            dt_s,
            derivative_function,
            post_step_routine);
      };"""


INTEGRATORS["rk4"] = integrator(
    "rk4",
    parse_rk4_integrator,
    include_rk4_integrator,
    construct_rk4_integrator,
)


def parse_integrator(simulation_config):
    path = "simulation"
    integrator_type = require_value(
        simulation_config, "integrator", INTEGRATORS.keys(), path
    )
    integrator_support = INTEGRATORS[integrator_type]
    return {
        "type": integrator_type,
        "data": integrator_support["parse"](simulation_config, path),
        "include": integrator_support["include"],
        "advance_state": integrator_support["advance_state"],
    }


def render_integrator_include(parsed_integrator):
    return parsed_integrator["include"](parsed_integrator["data"])


def render_integrator_advance_state(parsed_integrator):
    return parsed_integrator["advance_state"](parsed_integrator["data"])


# =============================================================================
# Atmosphere Models
# =============================================================================

ATMOSPHERE_MODELS = {}

# Constant atmosphere

def parse_constant_atmosphere(model_config, path):
    return {
        "density_kg_per_m3": require_number(
            model_config, "density_kg_per_m3", path
        ),
        "pressure_pa": require_number(model_config, "pressure_pa", path),
        "temperature_k": require_number(model_config, "temperature_k", path),
        "speed_of_sound_m_per_s": require_number(
            model_config, "speed_of_sound_m_per_s", path
        ),
    }


def include_constant_atmosphere(_parsed):
    return '#include "sim/models/atmosphere/constant_atmosphere.hpp"'


def construct_constant_atmosphere(parsed):
    return f"""  AtmosphereState atmosphere_state{{}};
  atmosphere_state.density_kg_per_m3 = {cpp_number(parsed["density_kg_per_m3"])};
  atmosphere_state.pressure_pa = {cpp_number(parsed["pressure_pa"])};
  atmosphere_state.temperature_k = {cpp_number(parsed["temperature_k"])};
  atmosphere_state.speed_of_sound_m_per_s = {cpp_number(parsed["speed_of_sound_m_per_s"])};
  const ConstantAtmosphere atmosphere_model{{atmosphere_state}};"""


ATMOSPHERE_MODELS["constant"] = model(
    "constant",
    parse_constant_atmosphere,
    include_constant_atmosphere,
    construct_constant_atmosphere,
)


# =============================================================================
# Wind Models
# =============================================================================

WIND_MODELS = {}

# Constant wind

def parse_constant_wind(model_config, path):
    return {
        "wind_ned_mps": require_vector(model_config, "wind_ned_mps", 3, path),
    }


def include_constant_wind(_parsed):
    return '#include "sim/models/wind/constant_wind.hpp"'


def construct_constant_wind(parsed):
    return f"""  const math::Vector3 wind_ned_mps{cpp_vector(parsed["wind_ned_mps"])};
  const ConstantWind wind_model{{wind_ned_mps}};"""


WIND_MODELS["constant"] = model(
    "constant",
    parse_constant_wind,
    include_constant_wind,
    construct_constant_wind,
)


# =============================================================================
# Gravity Models
# =============================================================================

GRAVITY_MODELS = {}

# Constant gravity

def parse_constant_gravity(model_config, path):
    return {
        "gravity_mps2": require_number(model_config, "gravity_mps2", path),
    }


def include_constant_gravity(_parsed):
    return '#include "sim/models/gravity/constant_gravity.hpp"'


def construct_constant_gravity(parsed):
    return f"""  const double gravity_mps2 = {cpp_number(parsed["gravity_mps2"])};
  const ConstantGravity gravity_model{{gravity_mps2}};"""


GRAVITY_MODELS["constant"] = model(
    "constant",
    parse_constant_gravity,
    include_constant_gravity,
    construct_constant_gravity,
)


# =============================================================================
# Aerodynamics Models
# =============================================================================

AERODYNAMICS_MODELS = {}

# Zero aerodynamics

def parse_zero_aerodynamics(_model_config, _path):
    return {}


def include_zero_aerodynamics(_parsed):
    return '#include "sim/models/aerodynamics/zero_aerodynamics.hpp"'


def construct_zero_aerodynamics(_parsed):
    return "  const ZeroAerodynamics aerodynamics_model{};"


AERODYNAMICS_MODELS["zero"] = model(
    "zero",
    parse_zero_aerodynamics,
    include_zero_aerodynamics,
    construct_zero_aerodynamics,
)


# SimpleAerodynamicsModelV1

def parse_simple_aerodynamics_model_v1(model_config, path):
    return {
        "reference_area_m2": require_number(
            model_config, "reference_area_m2", path
        ),
        "drag_coefficient": require_number(
            model_config, "drag_coefficient", path
        ),
        "fin_count": require_int(model_config, "fin_count", path),
        "fin_planform_area_m2": require_number(
            model_config, "fin_planform_area_m2", path
        ),
        "fin_normal_force_slope_per_rad": require_number(
            model_config, "fin_normal_force_slope_per_rad", path
        ),
        "nose_normal_force_slope_per_rad": require_number(
            model_config, "nose_normal_force_slope_per_rad", path
        ),
    }


def include_simple_aerodynamics_model_v1(_parsed):
    return '#include "sim/models/aerodynamics/simple_aerodynamics_model_v1.hpp"'


def construct_simple_aerodynamics_model_v1(parsed):
    return f"""  const SimpleAerodynamicsModelV1Params aerodynamics_params{{
      {cpp_number(parsed["reference_area_m2"])},
      {cpp_number(parsed["drag_coefficient"])},
      {parsed["fin_count"]},
      {cpp_number(parsed["fin_planform_area_m2"])},
      {cpp_number(parsed["fin_normal_force_slope_per_rad"])},
      {cpp_number(parsed["nose_normal_force_slope_per_rad"])},
  }};
  const SimpleAerodynamicsModelV1 aerodynamics_model{{aerodynamics_params}};"""


AERODYNAMICS_MODELS["simple_aerodynamics_model_v1"] = model(
    "simple_aerodynamics_model_v1",
    parse_simple_aerodynamics_model_v1,
    include_simple_aerodynamics_model_v1,
    construct_simple_aerodynamics_model_v1,
)


# =============================================================================
# Propulsion Models
# =============================================================================

PROPULSION_MODELS = {}

# Hardcoded thrust curve propulsion

def parse_hardcoded_thrust_curve_propulsion(model_config, path):
    return {
        "thrust_direction_body": require_vector(
            model_config, "thrust_direction_body", 3, path
        ),
    }


def include_hardcoded_thrust_curve_propulsion(_parsed):
    return '#include "sim/models/propulsion/hardcoded_thrust_curve_propulsion.hpp"'


def construct_hardcoded_thrust_curve_propulsion(parsed):
    return (
        "  const HardcodedThrustCurvePropulsion propulsion_model"
        f"{{{cpp_vector(parsed['thrust_direction_body'])}}};"
    )


PROPULSION_MODELS["hardcoded_thrust_curve"] = model(
    "hardcoded_thrust_curve",
    parse_hardcoded_thrust_curve_propulsion,
    include_hardcoded_thrust_curve_propulsion,
    construct_hardcoded_thrust_curve_propulsion,
)


# Zero propulsion

def parse_zero_propulsion(_model_config, _path):
    return {}


def include_zero_propulsion(_parsed):
    return '#include "sim/models/propulsion/zero_propulsion.hpp"'


def construct_zero_propulsion(_parsed):
    return "  const ZeroPropulsion propulsion_model{};"


PROPULSION_MODELS["zero"] = model(
    "zero",
    parse_zero_propulsion,
    include_zero_propulsion,
    construct_zero_propulsion,
)


MODEL_CATEGORIES = {
    "atmosphere": ATMOSPHERE_MODELS,
    "wind": WIND_MODELS,
    "gravity": GRAVITY_MODELS,
    "aerodynamics": AERODYNAMICS_MODELS,
    "propulsion": PROPULSION_MODELS,
}


def load_scenario(path):
    with path.open("r", encoding="utf-8") as stream:
        scenario = yaml.safe_load(stream)
    return require_mapping(scenario, "scenario")


def parse_scenario(scenario):
    simulation = require_mapping(scenario.get("simulation"), "simulation")
    logging_rate_hz = require_number(simulation, "logging_rate_hz", "simulation")
    if logging_rate_hz <= 0.0:
        raise ValueError("simulation.logging_rate_hz must be greater than zero")

    initial_state = require_mapping(scenario.get("initial_state"), "initial_state")
    rigid_body = require_mapping(
        initial_state.get("rigid_body"), "initial_state.rigid_body"
    )
    models = require_mapping(scenario.get("models"), "models")
    vehicle = require_mapping(scenario.get("vehicle"), "vehicle")
    mass_properties = require_mapping(
        vehicle.get("mass_properties"), "vehicle.mass_properties"
    )

    parsed_models = {}
    for category, support_table in MODEL_CATEGORIES.items():
        model_config = require_mapping(models.get(category), f"models.{category}")
        parsed_models[category] = parse_model(category, model_config, support_table)

    parsed_integrator = parse_integrator(simulation)
    q_body2ned = parse_initial_attitude(rigid_body)

    return {
        "dt_s": require_number(simulation, "dt_s", "simulation"),
        "logging_rate_hz": logging_rate_hz,
        "stop_simulation_time_s": require_number(
            simulation, "stop_simulation_time_s", "simulation"
        ),
        "position_ned_m": require_vector(
            rigid_body, "position_ned_m", 3, "initial_state.rigid_body"
        ),
        "velocity_body_mps": require_vector(
            rigid_body, "velocity_body_mps", 3, "initial_state.rigid_body"
        ),
        "omega_body_rps": require_vector(
            rigid_body, "omega_body_rps", 3, "initial_state.rigid_body"
        ),
        "q_body2ned": q_body2ned,
        "unloaded_mass_kg": require_number(
            mass_properties, "unloaded_mass_kg", "vehicle.mass_properties"
        ),
        "inertia_body_kgm2": require_vector(
            mass_properties, "inertia_body_kgm2", 3, "vehicle.mass_properties"
        ),
        "integrator": parsed_integrator,
        "models": parsed_models,
    }


def render_model_includes(config):
    return "\n".join(
        render_model_include(config["models"][category])
        for category in MODEL_CATEGORIES
    )


def render_model_constructions(config):
    return "\n\n".join(
        render_model_construction(config["models"][category])
        for category in MODEL_CATEGORIES
    )


def render_run_simulation(config):
    return f"""#include "sixsim/sim/run_simulation.hpp"

{render_integrator_include(config["integrator"])}
#include "sixsim/sim/logging.hpp"
#include "sixsim/sim/sim_general.hpp"

#include "sim/models/dynamics/rigid_body.hpp"
{render_model_includes(config)}

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace sixsim::sim {{

void log_truth_sample(LogSink& log,
                      const SimTime& time,
                      const RigidBodyState& state) {{
  const LogField fields[] = {{
      {{"position_ned_m.x", state.position_ned_m.x}},
      {{"position_ned_m.y", state.position_ned_m.y}},
      {{"position_ned_m.z", state.position_ned_m.z}},
      {{"velocity_body_mps.x", state.velocity_body_mps.x}},
      {{"velocity_body_mps.y", state.velocity_body_mps.y}},
      {{"velocity_body_mps.z", state.velocity_body_mps.z}},
      {{"q_body2ned.w", state.q_body2ned.w}},
      {{"q_body2ned.x", state.q_body2ned.x}},
      {{"q_body2ned.y", state.q_body2ned.y}},
      {{"q_body2ned.z", state.q_body2ned.z}},
      {{"omega_body_rps.x", state.omega_body_rps.x}},
      {{"omega_body_rps.y", state.omega_body_rps.y}},
      {{"omega_body_rps.z", state.omega_body_rps.z}},
  }};

  log.log_sample("truth", time, fields);
}}

void run_simulation(const std::filesystem::path& output_directory) {{
  SimulationConfig config{{}};
  config.dt_s = {cpp_number(config["dt_s"])};
  config.stop_simulation_time_s = {cpp_number(config["stop_simulation_time_s"])};

  SimulationEvents events{{}};
  SimTime time{{}};
  RigidBodyState state{{}};
  state.position_ned_m = {cpp_vector(config["position_ned_m"])};
  state.velocity_body_mps = {cpp_vector(config["velocity_body_mps"])};
  state.omega_body_rps = {cpp_vector(config["omega_body_rps"])};
  state.q_body2ned = {cpp_vector(config["q_body2ned"])};

{render_integrator_advance_state(config["integrator"])}

  const auto post_step_routine = post_step_rigid_body;

{render_model_constructions(config)}

  const ActuatorState actuator{{}};
  const std::filesystem::path selected_run_directory =
      output_directory.empty()
          ? std::filesystem::path{{{cpp_string(config["run_directory"])}}}
          : output_directory;
  const std::filesystem::path run_directory =
      std::filesystem::absolute(selected_run_directory).lexically_normal();
  const std::filesystem::path current_directory =
      std::filesystem::current_path().lexically_normal();
  if (run_directory == run_directory.root_path() ||
      run_directory == current_directory) {{
    throw std::runtime_error("refusing to replace unsafe run directory: " +
                             run_directory.string());
  }}
  const std::filesystem::path run_marker = run_directory / ".sixsim-run";
  if (std::filesystem::exists(run_directory)) {{
    std::ifstream marker_input(run_marker);
    std::string marker_value;
    std::getline(marker_input, marker_value);
    if (!marker_input || marker_value != "SixSim run directory" ||
        marker_input.peek() != std::char_traits<char>::eof()) {{
      throw std::runtime_error(
          "refusing to replace unmarked run directory: " +
          run_directory.string());
    }}
  }}
  std::filesystem::remove_all(run_directory);
  const std::filesystem::path config_directory = run_directory / "configs";
  const std::filesystem::path source_scenario_path =
      {cpp_string(config["scenario_path"])};
  const std::filesystem::path copied_scenario_path =
      config_directory / "scenario.yaml";
  std::filesystem::create_directories(config_directory);
  std::ofstream marker_output(run_marker);
  marker_output << "SixSim run directory\\n";
  if (!marker_output) {{
    throw std::runtime_error("failed to write run directory marker: " +
                             run_marker.string());
  }}
  std::filesystem::copy_file(
      source_scenario_path,
      copied_scenario_path,
      std::filesystem::copy_options::overwrite_existing);
  LogSink log{{run_directory}};
  const double logging_period_s =
      1.0 / {cpp_number(config["logging_rate_hz"])};
  double next_log_time_s = 0.0;

  const double unloaded_mass_kg = {cpp_number(config["unloaded_mass_kg"])};

  MassProperties mass_properties{{}};
  mass_properties.mass_kg = unloaded_mass_kg;
  mass_properties.inertia_body_kgm2 = {cpp_vector(config["inertia_body_kgm2"])};

  const auto compute_derivative =
      [&](const auto& current_state, const ForceMoment& force_moment) {{
        return rigid_body_derivative(
            current_state, force_moment, mass_properties);
      }};

  VehicleContext vehicle{{
      mass_properties,
      unloaded_mass_kg,
      actuator,
  }};

  const AuxiliaryContext auxiliary{{
      vehicle,
      atmosphere_model,
      wind_model,
      gravity_model,
      aerodynamics_model,
      propulsion_model,
  }};

  const auto step_state =
      [&](const SimTime& current_time, const auto& current_state, double dt_s) {{
        if (current_time.simtime_s >= next_log_time_s) {{
          log_truth_sample(log, current_time, current_state);
          do {{
            next_log_time_s += logging_period_s;
          }} while (next_log_time_s <= current_time.simtime_s);
        }}

        const AtmosphereState atmosphere =
            auxiliary.atmosphere_model.evaluate(current_time,
                                                current_state.position_ned_m);
        const WindState wind =
            auxiliary.wind_model.evaluate(current_time,
                                          current_state.position_ned_m);
        const GravityState gravity =
            auxiliary.gravity_model.evaluate(current_time,
                                             current_state.position_ned_m);
        const AerodynamicState aerodynamic_state =
            compute_aerodynamic_state(current_state, atmosphere, wind);

        const ForceMoment propulsion_force_moment =
            auxiliary.propulsion_model.evaluate(current_time,
                                                current_state,
                                                auxiliary.vehicle);
        const ForceMoment gravity_force_moment =
            gravity_force_moment_body(gravity,
                                      auxiliary.vehicle.mass_properties,
                                      current_state.q_body2ned);
        const ForceMoment aerodynamics_force_moment =
            auxiliary.aerodynamics_model.evaluate(current_state,
                                                  aerodynamic_state,
                                                  auxiliary.vehicle);

        const ForceMoment force_moment =
            combine_force_moment(gravity_force_moment,
                                 aerodynamics_force_moment,
                                 propulsion_force_moment);

        return advance_state_rk4(
            current_state,
            dt_s,
            [&](const auto& rk4_state) {{
              return compute_derivative(rk4_state, force_moment);
            }},
            post_step_routine);
      }};

  state = run_loop(config, step_state, state, events, time);

  std::ofstream manifest(run_directory / "manifest.yaml");
  if (!manifest) {{
    throw std::runtime_error("failed to open run manifest");
  }}
  manifest << "scenario:\\n"
           << "  original: " << std::quoted(source_scenario_path.string()) << '\\n'
           << "  copied: \\"configs/scenario.yaml\\"\\n"
           << "raw:\\n"
           << "  truth: \\"raw/truth.csv\\"\\n";
  if (!manifest) {{
    throw std::runtime_error("failed to write run manifest");
  }}

  std::cout << "final simtime: " << time.simtime_s << '\\n';
  std::cout << "stop simulation trigger time: "
            << events.stop_simulation.trigger_time_s << '\\n';
  std::cout << "final position NED z: " << std::fixed << std::setprecision(9)
            << state.position_ned_m.z << '\\n';
  std::cout << "final velocity body x: " << std::fixed << std::setprecision(9)
            << state.velocity_body_mps.x << '\\n';
  std::cout << "final velocity body y: " << std::fixed << std::setprecision(9)
            << state.velocity_body_mps.y << '\\n';
  std::cout << "final mass: " << std::fixed << std::setprecision(9)
            << mass_properties.mass_kg << '\\n';
}}

}}  // namespace sixsim::sim
"""


def main():
    parser = argparse.ArgumentParser(
        description="Generate SixSim run_simulation.cpp from a scenario YAML file."
    )
    parser.add_argument("scenario", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    try:
        scenario = load_scenario(args.scenario)
        config = parse_scenario(scenario)
        config["scenario_path"] = str(args.scenario.resolve())
        config["run_directory"] = f"runs/{args.scenario.stem}"
    except OSError as error:
        print(f"Failed to read {args.scenario}: {error}", file=sys.stderr)
        return 1
    except yaml.YAMLError as error:
        print(f"Invalid YAML {args.scenario}: {error}", file=sys.stderr)
        return 1
    except ValueError as error:
        print(f"Invalid scenario {args.scenario}: {error}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_run_simulation(config), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
