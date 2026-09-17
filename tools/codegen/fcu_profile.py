from pathlib import Path
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

FCU_CPP_TYPES = {
    "dummy_uno_r3": {
        "sitl_header": "hal/sitl/dummy_uno_r3/hal.hpp",
        "sitl_hal_type": "sixsim::hal::SitlDummyUnoR3Hal",
        "sitl_fcu_type": "sixsim::hal::SitlDummyUnoR3Fcu",
    },
}


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


def require_type(mapping, valid_types, path):
    actual = mapping.get("type")
    if actual not in valid_types:
        valid = ", ".join(sorted(repr(value) for value in valid_types))
        raise ValueError(f"{path}.type must be one of {valid}, got {actual!r}")
    return actual


def require_cpp_identifier(value, path):
    if (
        not isinstance(value, str)
        or CPP_IDENTIFIER_PATTERN.fullmatch(value) is None
        or value in CPP17_KEYWORDS
    ):
        raise ValueError(f"{path} must be a valid, non-keyword C++ identifier")
    return value


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
    if name not in FCU_CPP_TYPES:
        raise ValueError(f"FCU profile has no registered C++ types: {name}")
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
        device_type = require_type(device_config, {"serial"}, device_path)
        baud_rate = require_int(device_config, "baud_rate", device_path)
        if baud_rate <= 0 or baud_rate > 0xFFFFFFFF:
            raise ValueError(
                f"{device_path}.baud_rate must be between 1 and 4294967295"
            )
        parsed_devices[device_name] = {
            "type": device_type,
            "baud_rate": baud_rate,
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
        require_type(sensor_config, {"altimeter", "timer"}, sensor_path)
    return {
        "name": name,
        "base_tick_hz": base_tick_hz,
        "cycle_rate_hz": cycle_rate_hz,
        "devices": {**parsed_devices, "sensors": sensors},
        "cpp_types": FCU_CPP_TYPES[name],
    }
