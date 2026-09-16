#!/usr/bin/env python3

from pathlib import Path
import argparse

from fcu_profile import resolve_fcu_profile


def render_fcu_config_header(profile):
    return (
        "#pragma once\n\n"
        '#include "sixsim/flight/flight_computer.hpp"\n\n'
        "namespace sixsim::hal::generated {\n\n"
        "inline constexpr sixsim::flight::FlightTimingConfig flight_timing{\n"
        f"    {profile['base_tick_hz']},\n"
        f"    {profile['cycle_rate_hz']},\n"
        "};\n\n"
        "}  // namespace sixsim::hal::generated\n"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile")
    parser.add_argument("output")
    args = parser.parse_args()

    profile = resolve_fcu_profile(args.profile)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_fcu_config_header(profile), encoding="utf-8")


if __name__ == "__main__":
    main()
