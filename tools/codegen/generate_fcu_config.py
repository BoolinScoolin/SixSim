#!/usr/bin/env python3

from pathlib import Path
import argparse

from fcu_profile import resolve_fcu_profile


DEVICE_INTERFACES = {
    "serial": {
        "include": "sixsim/hal/serial.hpp",
        "registry_accessor": "serial",
        "type": "Serial",
    },
}

SENSOR_INTERFACES = {
    "altimeter": {
        "include": "sixsim/hal/altimeter.hpp",
        "type": "Altimeter",
    },
    "timer": {
        "include": "sixsim/hal/timer.hpp",
        "type": "Timer",
    },
}


def render_fcu_config_header(profile):
    return (
        "#pragma once\n\n"
        '#include "sixsim/flight/flight_computer.hpp"\n\n'
        "#include <stdint.h>\n\n"
        "namespace sixsim::hal::generated {\n\n"
        "inline constexpr sixsim::flight::FlightTimingConfig flight_timing{\n"
        f"    {profile['base_tick_hz']},\n"
        f"    {profile['cycle_rate_hz']},\n"
        "};\n\n"
        "inline constexpr uint32_t serial_baud_rate = "
        f"{profile['devices']['serial']['baud_rate']};\n\n"
        "}  // namespace sixsim::hal::generated\n"
    )


def render_sitl_devices_header(profile):
    sensors = profile["devices"]["sensors"]
    devices = {
        name: config
        for name, config in profile["devices"].items()
        if name != "sensors"
    }

    includes = {
        SENSOR_INTERFACES[config["type"]]["include"]
        for config in sensors.values()
    }
    includes.update(
        DEVICE_INTERFACES[config["type"]]["include"]
        for config in devices.values()
    )
    includes.add("hal/sitl/device_registry.hpp")

    lines = ["#pragma once", ""]
    lines.extend(f'#include "{include}"' for include in sorted(includes))
    namespace = f'sixsim::hal::generated::{profile["name"]}'
    lines.extend(("", f"namespace {namespace} {{", ""))

    sensors_type = "SitlSensors"
    lines.extend(
        (
            f"class {sensors_type} {{",
            " public:",
            f"  explicit {sensors_type}(",
            "      ::sixsim::hal::SitlSensors& available_sensors)",
        )
    )
    sensor_initializers = [
        f'{name}_(available_sensors.require<'
        f'{SENSOR_INTERFACES[config["type"]]["type"]}>("{name}"))'
        for name, config in sensors.items()
    ]
    for index, initializer in enumerate(sensor_initializers):
        punctuation = " {}" if index == len(sensor_initializers) - 1 else ","
        prefix = "      : " if index == 0 else "        "
        lines.append(f"{prefix}{initializer}{punctuation}")
    if not sensor_initializers:
        lines[-1] += " {}"
    lines.append("")
    for name, config in sensors.items():
        interface_type = SENSOR_INTERFACES[config["type"]]["type"]
        lines.append(f"  {interface_type}& {name}() {{ return {name}_; }}")
    lines.extend(("", " private:"))
    for name, config in sensors.items():
        interface_type = SENSOR_INTERFACES[config["type"]]["type"]
        lines.append(f"  {interface_type}& {name}_;")
    lines.extend(("};", ""))

    devices_type = "SitlDevices"
    lines.extend(
        (
            f"class {devices_type} {{",
            " public:",
            f"  explicit {devices_type}(",
            "      ::sixsim::hal::SitlDeviceRegistry& available_devices)",
        )
    )
    device_initializers = [
        "sensors_(available_devices.sensors())",
        *(
            f'{name}_(available_devices.'
            f'{DEVICE_INTERFACES[config["type"]]["registry_accessor"]}())'
            for name, config in devices.items()
        ),
    ]
    for index, initializer in enumerate(device_initializers):
        punctuation = " {}" if index == len(device_initializers) - 1 else ","
        prefix = "      : " if index == 0 else "        "
        lines.append(f"{prefix}{initializer}{punctuation}")
    lines.extend(
        (
            "",
            f"  {sensors_type}& sensors() {{ return sensors_; }}",
            f"  const {sensors_type}& sensors() const {{ return sensors_; }}",
        )
    )
    for name, config in devices.items():
        interface_type = DEVICE_INTERFACES[config["type"]]["type"]
        lines.append(f"  {interface_type}& {name}() {{ return {name}_; }}")
    lines.extend(("", " private:", f"  {sensors_type} sensors_;"))
    for name, config in devices.items():
        interface_type = DEVICE_INTERFACES[config["type"]]["type"]
        lines.append(f"  {interface_type}& {name}_;")
    lines.extend(("};", "", f"}}  // namespace {namespace}", ""))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sitl-devices", action="store_true")
    parser.add_argument("profile")
    parser.add_argument("output")
    args = parser.parse_args()

    profile = resolve_fcu_profile(args.profile)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rendered = (
        render_sitl_devices_header(profile)
        if args.sitl_devices
        else render_fcu_config_header(profile)
    )
    output.write_text(rendered, encoding="utf-8")


if __name__ == "__main__":
    main()
