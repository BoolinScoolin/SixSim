#!/usr/bin/env python3

from pathlib import Path
import argparse

from fcu_profile import (
    render_device_config_assignments,
    render_sensor_config_assignments,
    resolve_fcu_profile,
)


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
        "template <typename Devices>\n"
        "inline void configure_devices(Devices& devices) {\n"
        + "\n".join(
            f"  {line}"
            for line in (
                render_device_config_assignments(profile, "devices.")
                + render_sensor_config_assignments(profile, "devices.sensors().")
            )
        )
        + ("\n" if profile["devices"] else "")
        + "}\n\n"
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
        f"sixsim/hal/{config['type']}.hpp"
        for config in sensors.values()
    }
    includes.update(
        f"sixsim/hal/{config['type']}.hpp"
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
        f'::sixsim::hal::{config["type"]}>("{name}"))'
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
        interface_type = f"::sixsim::hal::{config['type']}"
        lines.append(f"  {interface_type}& {name}() {{ return {name}_; }}")
    lines.extend(("", " private:"))
    for name, config in sensors.items():
        interface_type = f"::sixsim::hal::{config['type']}"
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
            f'{name}_(available_devices.{config["type"]}())'
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
        interface_type = f"::sixsim::hal::{config['type']}"
        lines.append(f"  {interface_type}& {name}() {{ return {name}_; }}")
    lines.extend(("", " private:", f"  {sensors_type} sensors_;"))
    for name, config in devices.items():
        interface_type = f"::sixsim::hal::{config['type']}"
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
