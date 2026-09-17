#!/usr/bin/env python3

from pathlib import Path
import argparse
import math
import sys

from fcu_profile import resolve_fcu_profile

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


# Generate scenario_config.hpp manually:
#
#   python -m pip install -r tools/codegen/requirements.txt
#   python tools/codegen/generate_scenario_config.py configs/scenarios/test/smoke.yaml build/generated/smoke/scenario_config.hpp
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


def whole_ticks(seconds, tick_rate_hz, path):
    ticks = seconds * tick_rate_hz
    rounded_ticks = round(ticks)
    if not math.isclose(
        ticks, rounded_ticks, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError(
            f"{path} must represent a whole tick for the "
            f"{tick_rate_hz} Hz base tick"
        )
    if rounded_ticks <= 0:
        raise ValueError(f"{path} must be greater than zero")
    return rounded_ticks


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


def model(type_name, parse, include, construct, component, sources):
    return {
        "type": type_name,
        "parse": parse,
        "include": include,
        "construct": construct,
        "component": component,
        "sources": sources,
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
        "component": model_support["component"],
        "sources": model_support["sources"],
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
    return """inline auto advance_state(
    const auto& current_state,
    double dt_s,
    const auto& derivative_function,
    const auto& post_step_routine) {
  return step_rk4(
      current_state, dt_s, derivative_function, post_step_routine);
}"""


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
    return f"""  scenario.environment.atmosphere =
      std::make_unique<ConstantAtmosphere>(AtmosphereState{{
          {cpp_number(parsed["density_kg_per_m3"])},
          {cpp_number(parsed["pressure_pa"])},
          {cpp_number(parsed["temperature_k"])},
          {cpp_number(parsed["speed_of_sound_m_per_s"])},
      }});"""


ATMOSPHERE_MODELS["constant"] = model(
    "constant",
    parse_constant_atmosphere,
    include_constant_atmosphere,
    construct_constant_atmosphere,
    "sixsim_model_atmosphere_constant",
    ["sim/models/atmosphere/constant_atmosphere.cpp"],
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
    return f"""  scenario.environment.wind =
      std::make_unique<ConstantWind>(
          math::Vector3{cpp_vector(parsed["wind_ned_mps"])});"""


WIND_MODELS["constant"] = model(
    "constant",
    parse_constant_wind,
    include_constant_wind,
    construct_constant_wind,
    "sixsim_model_wind_constant",
    ["sim/models/wind/constant_wind.cpp"],
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
    return f"""  scenario.environment.gravity =
      std::make_unique<ConstantGravity>(
          {cpp_number(parsed["gravity_mps2"])});"""


GRAVITY_MODELS["constant"] = model(
    "constant",
    parse_constant_gravity,
    include_constant_gravity,
    construct_constant_gravity,
    "sixsim_model_gravity_constant",
    ["sim/models/gravity/constant_gravity.cpp"],
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
    return (
        "  vehicle.aerodynamics = "
        "std::make_unique<ZeroAerodynamics>();"
    )


AERODYNAMICS_MODELS["zero"] = model(
    "zero",
    parse_zero_aerodynamics,
    include_zero_aerodynamics,
    construct_zero_aerodynamics,
    "sixsim_model_aerodynamics_zero",
    ["sim/models/aerodynamics/zero_aerodynamics.cpp"],
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
    return f"""  vehicle.aerodynamics =
      std::make_unique<SimpleAerodynamicsModelV1>(
          SimpleAerodynamicsModelV1Params{{
              {cpp_number(parsed["reference_area_m2"])},
              {cpp_number(parsed["drag_coefficient"])},
              {parsed["fin_count"]},
              {cpp_number(parsed["fin_planform_area_m2"])},
              {cpp_number(parsed["fin_normal_force_slope_per_rad"])},
              {cpp_number(parsed["nose_normal_force_slope_per_rad"])},
          }});"""


AERODYNAMICS_MODELS["simple_aerodynamics_model_v1"] = model(
    "simple_aerodynamics_model_v1",
    parse_simple_aerodynamics_model_v1,
    include_simple_aerodynamics_model_v1,
    construct_simple_aerodynamics_model_v1,
    "sixsim_model_aerodynamics_simple_v1",
    ["sim/models/aerodynamics/simple_aerodynamics_model_v1.cpp"],
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
    return f"""  vehicle.propulsion =
      std::make_unique<HardcodedThrustCurvePropulsion>(
          math::Vector3{cpp_vector(parsed["thrust_direction_body"])});"""


PROPULSION_MODELS["hardcoded_thrust_curve"] = model(
    "hardcoded_thrust_curve",
    parse_hardcoded_thrust_curve_propulsion,
    include_hardcoded_thrust_curve_propulsion,
    construct_hardcoded_thrust_curve_propulsion,
    "sixsim_model_propulsion_hardcoded_thrust_curve",
    ["sim/models/propulsion/hardcoded_thrust_curve_propulsion.cpp"],
)


# Zero propulsion

def parse_zero_propulsion(_model_config, _path):
    return {}


def include_zero_propulsion(_parsed):
    return '#include "sim/models/propulsion/zero_propulsion.hpp"'


def construct_zero_propulsion(_parsed):
    return (
        "  vehicle.propulsion = "
        "std::make_unique<ZeroPropulsion>();"
    )


PROPULSION_MODELS["zero"] = model(
    "zero",
    parse_zero_propulsion,
    include_zero_propulsion,
    construct_zero_propulsion,
    "sixsim_model_propulsion_zero",
    ["sim/models/propulsion/zero_propulsion.cpp"],
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
    environment = require_mapping(scenario.get("environment", {}), "environment")
    origin_altitude_msl_m = (
        require_number(environment, "origin_altitude_msl_m", "environment")
        if "origin_altitude_msl_m" in environment
        else 0.0
    )
    simulation = require_mapping(scenario.get("simulation"), "simulation")
    dt_s = require_number(simulation, "dt_s", "simulation")
    logging_rate_hz = require_number(simulation, "logging_rate_hz", "simulation")
    if logging_rate_hz <= 0.0:
        raise ValueError("simulation.logging_rate_hz must be greater than zero")

    initial_state = require_mapping(scenario.get("initial_state"), "initial_state")
    rigid_body = require_mapping(
        initial_state.get("rigid_body"), "initial_state.rigid_body"
    )
    models = require_mapping(scenario.get("models"), "models")
    vehicle = require_mapping(scenario.get("vehicle"), "vehicle")
    vehicle_name = vehicle.get("name", "vehicle_0")
    if not isinstance(vehicle_name, str) or not vehicle_name.strip():
        raise ValueError("vehicle.name must be a non-empty string")
    fcu_names = vehicle.get("fcus", [])
    if not isinstance(fcu_names, list):
        raise ValueError("vehicle.fcus must be a list of profile names")
    resolved_fcu_profiles = []
    for index, fcu_name in enumerate(fcu_names):
        if not isinstance(fcu_name, str) or not fcu_name.strip():
            raise ValueError(f"vehicle.fcus[{index}] must be a non-empty string")
        resolved_fcu_profiles.append(resolve_fcu_profile(fcu_name))
    for profile in resolved_fcu_profiles:
        step_ticks = whole_ticks(
            dt_s,
            profile["base_tick_hz"],
            f"simulation.dt_s for FCU profile {profile['name']}",
        )
        if profile["base_tick_hz"] % profile["cycle_rate_hz"] != 0:
            raise ValueError(
                f"FCU profile {profile['name']}.cycle_rate_hz must produce a "
                f"whole-tick cycle period at {profile['base_tick_hz']} Hz"
            )
        cycle_period_ticks = profile["base_tick_hz"] // profile["cycle_rate_hz"]
        if cycle_period_ticks % step_ticks != 0:
            raise ValueError(
                f"simulation.dt_s ({step_ticks} ticks) must divide the "
                f"{profile['name']} flight cycle period "
                f"({cycle_period_ticks} ticks)"
            )
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
        "origin_altitude_msl_m": origin_altitude_msl_m,
        "vehicle_name": vehicle_name,
        "fcu_profiles": resolved_fcu_profiles,
        "dt_s": dt_s,
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


def render_model_constructions(config, categories):
    return "\n\n".join(
        render_model_construction(config["models"][category])
        for category in categories
    )


def render_sitl_sensor_includes(config):
    sensor_types = {
        sensor_config["type"]
        for profile in config["fcu_profiles"]
        for sensor_config in profile["devices"]["sensors"].values()
    }
    includes = []
    if "altimeter" in sensor_types:
        includes.append('#include "hal/sitl/altimeter.hpp"')
    if "timer" in sensor_types:
        includes.append('#include "hal/sitl/timer.hpp"')
    return "\n".join(includes)


def render_sitl_fcu_includes(config):
    includes = []
    for profile in config["fcu_profiles"]:
        include = profile["cpp_types"]["sitl_header"]
        if include not in includes:
            includes.append(include)
    return "\n".join(f'#include "{include}"' for include in includes)


def render_configured_fcu_types(config):
    fcu_types = [
        profile["cpp_types"]["sitl_fcu_type"]
        for profile in config["fcu_profiles"]
    ]
    if not fcu_types:
        return "using ConfiguredFcus = std::tuple<>;"
    return "using ConfiguredFcus = std::tuple<\n    " + ",\n    ".join(
        fcu_types
    ) + "\n>;"


def render_fcu_constructions(config):
    lines = []
    for index, profile in enumerate(config["fcu_profiles"]):
        registry = f"fcu_{index}_devices"
        lines.append(
            f"  auto {registry} = "
            "std::make_unique<hal::SitlDeviceRegistry>();"
        )
        for sensor_name, sensor_config in profile["devices"]["sensors"].items():
            if sensor_config["type"] == "altimeter":
                lines.append(
                    f"  {registry}->sensors()"
                    f'.create_sensor<hal::SitlAltimeter>'
                    f'({cpp_string(sensor_name)});'
                )
            elif sensor_config["type"] == "timer":
                lines.append(
                    f"  {registry}->sensors()"
                    f'.create_sensor<hal::SitlTimer>'
                    f'({cpp_string(sensor_name)});'
                )
        lines.append("")

    lines.append("  return ConfiguredFcus{")
    for index, profile in enumerate(config["fcu_profiles"]):
        cpp_types = profile["cpp_types"]
        lines.extend(
            (
                f'      {cpp_types["sitl_fcu_type"]}{{',
                "          flight::FlightTimingConfig{",
                f'              {profile["base_tick_hz"]},',
                f'              {profile["cycle_rate_hz"]}}},',
                f'          {cpp_types["sitl_hal_type"]}{{',
                f"              std::move(fcu_{index}_devices)}}}},",
            )
        )
    lines.append("  };")
    return "\n".join(lines)


def render_build_manifest(config):
    lines = [
        "# Generated by generate_scenario_config.py. Do not edit.",
        "",
    ]
    components = []
    for category in MODEL_CATEGORIES:
        model_config = config["models"][category]
        component = model_config["component"]
        if component in components:
            continue
        components.append(component)
        lines.append(f"sixsim_add_model_component({component}")
        for source in model_config["sources"]:
            lines.append(f'  "${{CMAKE_SOURCE_DIR}}/{source}"')
        lines.extend((")", ""))

    lines.append("set(SIXSIM_SCENARIO_MODEL_COMPONENTS")
    lines.extend(f"  {component}" for component in components)
    lines.extend((")", ""))

    lines.append("set(SIXSIM_SCENARIO_FCU_PROFILES")
    profile_names = []
    for profile in config["fcu_profiles"]:
        if profile["name"] not in profile_names:
            profile_names.append(profile["name"])
    lines.extend(f"  {profile_name}" for profile_name in profile_names)
    lines.extend((")", ""))

    lines.append("set(SIXSIM_SCENARIO_CONFIGURATION_DEPENDENCIES")
    for profile in config["fcu_profiles"]:
        lines.append(
            '  "${CMAKE_SOURCE_DIR}/configs/fcus/'
            f'{profile["name"]}.yaml"'
        )
    lines.extend((")", ""))
    return "\n".join(lines)


def render_scenario_config(config):
    return f"""#pragma once

{render_integrator_include(config["integrator"])}
#include "sixsim/flight/flight_computer.hpp"
#include "sixsim/sim/scenario.hpp"
{render_sitl_fcu_includes(config)}
{render_sitl_sensor_includes(config)}

{render_model_includes(config)}

#include <memory>
#include <tuple>
#include <utility>

namespace sixsim::sim {{

{render_integrator_advance_state(config["integrator"])}

{render_configured_fcu_types(config)}
using ConfiguredVehicle = Vehicle<ConfiguredFcus>;

inline Scenario build_scenario() {{
  Scenario scenario{{}};
  scenario.environment.origin_altitude_msl_m =
      {cpp_number(config["origin_altitude_msl_m"])};
  scenario.simulation.dt_s = {cpp_number(config["dt_s"])};
  scenario.simulation.stop_simulation_time_s =
      {cpp_number(config["stop_simulation_time_s"])};
  scenario.logging_rate_hz = {cpp_number(config["logging_rate_hz"])};
  scenario.source_scenario_path = {cpp_string(config["scenario_path"])};
  scenario.default_run_directory = {cpp_string(config["run_directory"])};

{render_model_constructions(config, ("atmosphere", "wind", "gravity"))}

  return scenario;
}}

inline ConfiguredFcus build_fcus() {{
{render_fcu_constructions(config)}
}}

inline ConfiguredVehicle build_vehicle() {{
  ConfiguredVehicle vehicle{{build_fcus()}};
  vehicle.name = {cpp_string(config["vehicle_name"])};
  vehicle.state.position_ned_m =
      {cpp_vector(config["position_ned_m"])};
  vehicle.state.velocity_body_mps =
      {cpp_vector(config["velocity_body_mps"])};
  vehicle.state.omega_body_rps =
      {cpp_vector(config["omega_body_rps"])};
  vehicle.state.q_body2ned = {cpp_vector(config["q_body2ned"])};
  vehicle.unloaded_mass_kg =
      {cpp_number(config["unloaded_mass_kg"])};
  vehicle.mass_properties.mass_kg =
      vehicle.unloaded_mass_kg;
  vehicle.mass_properties.inertia_body_kgm2 =
      {cpp_vector(config["inertia_body_kgm2"])};
  vehicle.actuator = ActuatorState{{}};

{render_model_constructions(config, ("aerodynamics", "propulsion"))}

  return vehicle;
}}

}}  // namespace sixsim::sim
"""


def main():
    parser = argparse.ArgumentParser(
        description="Generate SixSim scenario build or C++ configuration."
    )
    parser.add_argument(
        "--build-manifest",
        action="store_true",
        help="generate a configure-time CMake build manifest",
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
    rendered = (
        render_build_manifest(config)
        if args.build_manifest
        else render_scenario_config(config)
    )
    args.output.write_text(rendered, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
