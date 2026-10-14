from pathlib import Path
import math
import re
import sys

try:
    import yaml
except ModuleNotFoundError:
    print(
        "Missing dependency: PyYAML. Install with "
        "`python3 -m pip install -r tools/codegen/requirements.txt`.",
        file=sys.stderr,
    )
    raise SystemExit(1)


REPO_ROOT = Path(__file__).resolve().parents[2]
CPP_IDENTIFIER_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
CPP17_KEYWORDS = frozenset(
    "alignas alignof and and_eq asm auto bitand bitor bool break case catch "
    "char char16_t char32_t class compl const constexpr const_cast continue "
    "decltype default delete do double dynamic_cast else enum explicit export "
    "extern false float for friend goto if inline int long mutable namespace "
    "new noexcept not not_eq nullptr operator or or_eq private protected public "
    "register reinterpret_cast return short signed sizeof static static_assert "
    "static_cast struct switch template this thread_local throw true try typedef "
    "typeid typename union unsigned using virtual void volatile wchar_t while "
    "xor xor_eq".split()
)

def require_mapping(value, path):
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def require_int(mapping, key, path):
    if key not in mapping:
        raise ValueError(f"{path}.{key} is required")
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{path}.{key} must be an integer")
    return value


def require_cpp_identifier(value, path):
    if (
        not isinstance(value, str)
        or CPP_IDENTIFIER_PATTERN.fullmatch(value) is None
        or value in CPP17_KEYWORDS
    ):
        raise ValueError(f"{path} must be a valid, non-keyword C++ identifier")
    return value


def cpp_literal(value):
    if value["datatype"] == "int":
        return str(value["value"])
    return repr(float(value["value"]))


def cpp_typed_literal(value):
    datatype = value["datatype"]
    actual_value = value["value"]
    if datatype in {"int", "double"}:
        return cpp_literal(value)
    cpp_types = {
        "vector3": "sixsim::Vector3",
        "quaternion": "sixsim::Quaternion",
        "matrix3": "sixsim::Matrix3",
    }
    if datatype in cpp_types:
        values = ", ".join(repr(float(item)) for item in actual_value)
        if datatype == "matrix3":
            return f"{cpp_types[datatype]}{{{{{{{values}}}}}}}"
        return f"{cpp_types[datatype]}{{{values}}}"
    raise ValueError(f"unsupported C++ config datatype: {datatype}")


def parse_typed_value(value, datatype, path):
    if datatype == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{path} must be an integer")
        return value
    if datatype == "double":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{path} must be a number")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"{path} must be finite")
        return parsed

    lengths = {"vector3": 3, "quaternion": 4, "matrix3": 9}
    if datatype in lengths:
        length = lengths[datatype]
        if not isinstance(value, list) or len(value) != length:
            raise ValueError(f"{path} must be a list of {length} numbers")
        if not all(
            not isinstance(item, bool) and isinstance(item, (int, float))
            for item in value
        ):
            raise ValueError(f"{path} must be a list of {length} numbers")
        parsed = [float(item) for item in value]
        if not all(math.isfinite(item) for item in parsed):
            raise ValueError(f"{path} must contain only finite numbers")
        if datatype == "quaternion":
            magnitude = math.sqrt(sum(item * item for item in parsed))
            if not math.isclose(magnitude, 1.0, rel_tol=0.0, abs_tol=1e-6):
                raise ValueError(f"{path} must be a unit quaternion")
        return parsed

    raise ValueError(
        f"{path} datatype must be one of 'double', 'int', 'matrix3', "
        "'quaternion', 'vector3'"
    )


def parse_config_value(value, path):
    value = require_mapping(value, path)
    if "datatype" not in value:
        raise ValueError(f"{path}.datatype is required")
    datatype = value.get("datatype")
    if datatype not in {"int", "double", "vector3", "quaternion", "matrix3"}:
        raise ValueError(
            f"{path}.datatype must be one of 'double', 'int', 'matrix3', "
            "'quaternion', 'vector3'"
        )
    if "value" not in value:
        raise ValueError(f"{path}.value is required")
    if value["value"] == "set_by_scenario":
        return {"datatype": datatype, "value": "set_by_scenario"}
    actual_value = parse_typed_value(value["value"], datatype, f"{path}.value")
    return {
        "datatype": datatype,
        "value": actual_value,
    }


def render_device_config_assignments(profile, devices_expression):
    lines = []
    for device_name, device in profile["devices"].items():
        if device_name == "sensors":
            continue
        for config_name, value in device["config"].items():
            if value["value"] == "set_by_scenario":
                continue
            lines.append(
                f"{devices_expression}{device_name}()"
                f".config().{config_name} = {cpp_typed_literal(value)};"
            )
    return lines


def render_sensor_config_assignments(profile, sensors_expression):
    lines = []
    sensors = profile["devices"]["sensors"]
    for sensor_name, sensor in sensors.items():
        for config_name, value in sensor["config"].items():
            if value["value"] == "set_by_scenario":
                continue
            lines.append(
                f"{sensors_expression}{sensor_name}()"
                f".config().{config_name} = {cpp_typed_literal(value)};"
            )
    return lines


def render_scenario_fcu_config_assignments(profile, devices_expression):
    lines = []

    def append_assignments(owner, owner_name, owner_kind):
        scenario_values = owner.get("scenario_config", {})
        config_layers = (
            ("common", owner["config"]),
            ("sitl", owner["sitl_config"]),
        )
        for scope, declarations in config_layers:
            for config_name, declaration in declarations.items():
                value = declaration
                if declaration["value"] == "set_by_scenario":
                    scenario_value = scenario_values[config_name]
                    value = {
                        "datatype": declaration["datatype"],
                        "value": scenario_value["value"],
                    }

                type_name = owner["type"]
                if scope == "sitl":
                    type_name = f"Sitl_{type_name}"
                if owner_kind == "sensor":
                    target = (
                        f'{devices_expression}sensors().require<'
                        f'::sixsim::hal::{type_name}>("{owner_name}")'
                    )
                else:
                    target = (
                        f"static_cast<::sixsim::hal::{type_name}&>("
                        f"{devices_expression}{owner_name}())"
                    )
                lines.append(
                    f"{target}.config().{config_name} = "
                    f"{cpp_typed_literal(value)};"
                )

    for device_name, device in profile["devices"].items():
        if device_name == "sensors":
            for sensor_name, sensor in device.items():
                append_assignments(sensor, sensor_name, "sensor")
        else:
            append_assignments(device, device_name, "device")
    return lines


def resolve_fcu_profile(name):
    profile_path = REPO_ROOT / "configs" / "fcus" / f"{name}.yaml"
    if not profile_path.is_file():
        raise ValueError(f"FCU profile does not exist: {profile_path}")
    try:
        with profile_path.open("r", encoding="utf-8") as stream:
            profile = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f"Failed to read FCU profile {profile_path}: {error}")
    profile = require_mapping(profile, f"FCU profile {name}")
    profile_name = profile.get("name")
    if profile_name != name:
        raise ValueError(
            f"FCU profile {profile_path}.name must match reference {name!r}"
        )
    require_cpp_identifier(profile_name, f"FCU profile {name}.name")
    cycle_rate_hz = require_int(profile, "cycle_rate_hz", f"FCU profile {name}")
    if cycle_rate_hz <= 0:
        raise ValueError(f"FCU profile {name}.cycle_rate_hz must be greater than zero")
    base_tick_hz = require_int(profile, "base_tick_hz", f"FCU profile {name}")
    if base_tick_hz <= 0:
        raise ValueError(f"FCU profile {name}.base_tick_hz must be greater than zero")
    devices = require_mapping(
        profile.get("devices"), f"FCU profile {name}.devices"
    )
    parsed_devices = {}
    for device_name, device_config in devices.items():
        if device_name == "sensors":
            continue
        require_cpp_identifier(device_name, f"FCU profile {name}.devices key")
        device_path = f"FCU profile {name}.devices.{device_name}"
        device_config = require_mapping(device_config, device_path)
        device_type = device_config.get("type")
        require_cpp_identifier(device_type, f"{device_path}.type")
        interface_path = (
            REPO_ROOT / "hal" / "include" / "sixsim" / "hal"
            / f"{device_type}.hpp"
        )
        sitl_path = REPO_ROOT / "hal" / "sitl" / f"{device_type}.hpp"
        if not interface_path.is_file():
            raise ValueError(
                f"{device_path}.type has no interface header: {interface_path}"
            )
        if not sitl_path.is_file():
            raise ValueError(
                f"{device_path}.type has no SITL implementation: {sitl_path}"
            )
        config = device_config.get("config", {})
        config = require_mapping(config, f"{device_path}.config")
        for config_name in config:
            require_cpp_identifier(
                config_name, f"{device_path}.config key"
            )
        config = {
            config_name: parse_config_value(
                config_value,
                f"{device_path}.config.{config_name}",
            )
            for config_name, config_value in config.items()
        }
        sitl_config = require_mapping(
            device_config.get("sitl", {}), f"{device_path}.sitl"
        )
        unknown_sitl_keys = set(sitl_config) - {"config"}
        if unknown_sitl_keys:
            unknown_key = sorted(unknown_sitl_keys)[0]
            raise ValueError(
                f"{device_path}.sitl.{unknown_key} is not a supported field"
            )
        sitl_config_values = require_mapping(
            sitl_config.get("config", {}), f"{device_path}.sitl.config"
        )
        for config_name in sitl_config_values:
            require_cpp_identifier(config_name, f"{device_path}.sitl.config key")
        sitl_config_values = {
            config_name: parse_config_value(
                config_value,
                f"{device_path}.sitl.config.{config_name}",
            )
            for config_name, config_value in sitl_config_values.items()
        }
        parsed_devices[device_name] = {
            "type": device_type,
            "config": config,
            "sitl_config": sitl_config_values,
        }
    sensors = devices.get("sensors", {})
    if not isinstance(sensors, dict):
        raise ValueError(f"FCU profile {name}.devices.sensors must be a mapping")
    for sensor_name, sensor_config in sensors.items():
        require_cpp_identifier(
            sensor_name, f"FCU profile {name}.devices.sensors key"
        )
        sensor_path = f"FCU profile {name}.devices.sensors.{sensor_name}"
        sensor_config = require_mapping(sensor_config, sensor_path)
        sensor_type = sensor_config.get("type")
        require_cpp_identifier(sensor_type, f"{sensor_path}.type")
        interface_path = (
            REPO_ROOT / "hal" / "include" / "sixsim" / "hal"
            / f"{sensor_type}.hpp"
        )
        sitl_path = REPO_ROOT / "hal" / "sitl" / f"{sensor_type}.hpp"
        if not interface_path.is_file():
            raise ValueError(
                f"{sensor_path}.type has no interface header: {interface_path}"
            )
        if not sitl_path.is_file():
            raise ValueError(
                f"{sensor_path}.type has no SITL implementation: {sitl_path}"
            )
        common_config_values = require_mapping(
            sensor_config.get("config", {}), f"{sensor_path}.config"
        )
        sitl_config = require_mapping(
            sensor_config.get("sitl", {}), f"{sensor_path}.sitl"
        )
        unknown_sitl_keys = set(sitl_config) - {"config"}
        if unknown_sitl_keys:
            unknown_key = sorted(unknown_sitl_keys)[0]
            raise ValueError(
                f"{sensor_path}.sitl.{unknown_key} is not a supported field"
            )
        sitl_config_values = require_mapping(
            sitl_config.get("config", {}), f"{sensor_path}.sitl.config"
        )
        for config_name in (*common_config_values, *sitl_config_values):
            require_cpp_identifier(config_name, f"{sensor_path}.config key")
        sensors[sensor_name] = {
            "type": sensor_type,
            "config": {
                config_name: parse_config_value(
                    config_value,
                    f"{sensor_path}.config.{config_name}",
                )
                for config_name, config_value in common_config_values.items()
            },
            "sitl_config": {
                config_name: parse_config_value(
                    config_value,
                    f"{sensor_path}.sitl.config.{config_name}",
                )
                for config_name, config_value in sitl_config_values.items()
            },
        }
    return {
        "name": name,
        "base_tick_hz": base_tick_hz,
        "cycle_rate_hz": cycle_rate_hz,
        "devices": {**parsed_devices, "sensors": sensors},
    }
