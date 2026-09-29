#!/usr/bin/env python3

from pathlib import Path
import argparse
import re

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CPP_IDENTIFIER_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
YAML_TYPES = {
    "int": ("int", "i32"),
    "double": ("double", "f64"),
    "bool": ("bool", "bool"),
    "v3": ("sixsim::Vector3", "v3"),
    "quat": ("sixsim::Quaternion", "quat"),
}


def require_identifier(value, description):
    if not isinstance(value, str) or CPP_IDENTIFIER_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{description} must be a C++ identifier")
    return value


def require_fields(algorithm, direction):
    fields = algorithm.get(direction)
    if not isinstance(fields, dict):
        raise ValueError(f"algorithm {direction} must be a mapping")

    for name, datatype in fields.items():
        require_identifier(name, f"algorithm {direction} field name")
        if datatype not in YAML_TYPES:
            raise ValueError(
                f"algorithm {direction}.{name} has unsupported type "
                f"{datatype!r}; supported types are int, double, bool, v3, quat"
            )
    return fields


def render_algorithm_header(name, inputs, outputs):
    types = set(inputs.values()) | set(outputs.values())
    lines = [
        "#pragma once",
        "",
    ]
    if "v3" in types or "quat" in types:
        lines.extend(('#include "sixsim/math.hpp"', ""))
    lines.extend(
        (
            "namespace sixsim::flight::algorithms {",
            "",
            f"class {name} {{",
            " public:",
            "  struct Inputs {",
            "    // Inputs",
        )
    )
    lines.extend(
        f"    {YAML_TYPES[datatype][0]} {field}{{}};"
        for field, datatype in inputs.items()
    )
    lines.extend(("  };", "", "  struct Outputs {", "    // Outputs"))
    lines.extend(
        f"    {YAML_TYPES[datatype][0]} {field}{{}};"
        for field, datatype in outputs.items()
    )
    lines.extend(
        (
            "  };",
            "",
            "  Inputs inputs;",
            "  Outputs outputs;",
            "",
            "  void enter();",
            "  void run();",
            "  void exit();",
            "};",
            "",
            "}  // namespace sixsim::flight::algorithms",
            "",
        )
    )
    return "\n".join(lines)


def render_mapping_header(algorithm_yaml, revision, output_root):
    algorithm_path = Path(algorithm_yaml).resolve()
    with algorithm_path.open(encoding="utf-8") as stream:
        algorithm = yaml.safe_load(stream)
    if not isinstance(algorithm, dict):
        raise ValueError("algorithm YAML must contain a mapping")

    name = require_identifier(algorithm.get("name"), "algorithm name")
    if algorithm_path.stem != name:
        raise ValueError(
            f"algorithm YAML filename {algorithm_path.stem!r} must match "
            f"its name {name!r}"
        )
    revision = require_identifier(revision, "revision name")

    inputs = require_fields(algorithm, "inputs")
    outputs = require_fields(algorithm, "outputs")
    if not inputs:
        raise ValueError("algorithm inputs must not be empty")
    if not outputs:
        raise ValueError("algorithm outputs must not be empty")

    common_values_header = (
        REPO_ROOT
        / "flight"
        / "src"
        / "profiles"
        / revision
        / "common_values.hpp"
    )
    if not common_values_header.is_file():
        raise ValueError(
            f"CommonValues header does not exist for revision {revision}: "
            f"{common_values_header}"
        )

    output_root = Path(output_root)
    if not output_root.is_absolute():
        output_root = REPO_ROOT / output_root
    output_root = output_root.resolve()
    try:
        generated_algorithm_header = (
            output_root / "include" / "sixsim" / "flight" / "algorithms"
            / f"{name}.hpp"
        ).relative_to(REPO_ROOT)
    except ValueError as error:
        raise ValueError("output root must be inside the repository") from error

    algorithm_include = generated_algorithm_header.as_posix()
    common_values_include = common_values_header.relative_to(REPO_ROOT).as_posix()
    profile_namespace = f"sixsim::flight::{revision}"
    generated_namespace = profile_namespace

    lines = [
        "#pragma once",
        "",
        f'#include "{algorithm_include}"',
        f'#include "{common_values_include}"',
        '#include "sixsim/flight/algorithm_io.hpp"',
        "",
        f"namespace {generated_namespace} {{",
        "",
        f"inline algorithms::{name}::Inputs get{name}Inputs(",
        f"    const ::{profile_namespace}::CommonValues& common) {{",
        f"  algorithms::{name}::Inputs inputs{{}};",
    ]
    lines.extend(
        f"  read_{YAML_TYPES[datatype][1]}(inputs.{field}, common.{field});"
        for field, datatype in inputs.items()
    )
    lines.extend(
        (
            "  return inputs;",
            "}",
            "",
            f"inline void write{name}Outputs(",
            f"    ::{profile_namespace}::CommonValues& common,",
            f"    const algorithms::{name}::Outputs& outputs) {{",
        )
    )
    lines.extend(
        f"  write_{YAML_TYPES[datatype][1]}(common.{field}, outputs.{field});"
        for field, datatype in outputs.items()
    )
    lines.extend(
        (
            "}",
            "",
            f"inline void run{name}(::{profile_namespace}::CommonValues& common,",
            f"                       algorithms::{name}& algorithm) {{",
            f"  algorithm.inputs = get{name}Inputs(common);",
            "  algorithm.run();",
            f"  write{name}Outputs(common, algorithm.outputs);",
            "}",
            "",
            f"}}  // namespace {generated_namespace}",
            "",
        )
    )
    return "\n".join(lines)


def read_state_manifest(manifest_path):
    with Path(manifest_path).open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    if not isinstance(manifest, dict) or not isinstance(manifest.get("states"), list):
        raise ValueError("state manifest must contain a states list")

    states = []
    seen_states = set()
    for state in manifest["states"]:
        if not isinstance(state, dict):
            raise ValueError("each state must be a mapping")
        name = require_identifier(state.get("name"), "state name")
        if name in seen_states:
            raise ValueError(f"duplicate state name {name!r}")
        seen_states.add(name)

        algorithms = state.get("algorithms")
        if not isinstance(algorithms, list):
            raise ValueError(f"state {name} algorithms must be a list")
        for algorithm in algorithms:
            require_identifier(algorithm, f"state {name} algorithm name")
        states.append((name, algorithms))

    if not states:
        raise ValueError("state manifest must list at least one state")
    return states


def render_state_enum(states, revision):
    lines = [
        "#pragma once",
        "",
        f"namespace sixsim::flight::{revision}::generated {{",
        "",
        "enum class State {",
    ]
    lines.extend(f"  {name}," for name, _ in states)
    lines.extend(
        (
            "};",
            "",
            f"}}  // namespace sixsim::flight::{revision}::generated",
            "",
        )
    )
    return "\n".join(lines)


def manifest_algorithm_names(states):
    algorithms_directory = REPO_ROOT / "flight" / "src" / "algorithms"
    names = []
    for _, algorithms in states:
        for name in algorithms:
            if name == "HalAdapter":
                continue

            algorithm_yaml = algorithms_directory / f"{name}.yaml"
            if not algorithm_yaml.is_file():
                raise ValueError(
                    f"manifest algorithm {name!r} has no matching YAML file: "
                    f"{algorithm_yaml}"
                )

            if name not in names:
                names.append(name)
    return names


def render_profile_bindings_header(states, revision, output_root):
    revision = require_identifier(revision, "revision name")
    output_root = Path(output_root)
    if not output_root.is_absolute():
        output_root = REPO_ROOT / output_root
    output_root = output_root.resolve()

    lines = ["#pragma once", ""]
    for name in manifest_algorithm_names(states):
        mapping_header = (
            output_root / "include" / "sixsim" / "flight" / "profiles"
            / revision / f"{name}Bindings.hpp"
        )
        try:
            mapping_include = mapping_header.relative_to(REPO_ROOT).as_posix()
        except ValueError as error:
            raise ValueError("output root must be inside the repository") from error
        lines.append(f'#include "{mapping_include}"')
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--algorithm-header-only", action="store_true")
    mode.add_argument("--mapping-only", action="store_true")
    mode.add_argument("--manifest-algorithms", action="store_true")
    mode.add_argument("--state-enum-only", action="store_true")
    mode.add_argument("--profile-bindings-only", action="store_true")
    parser.add_argument("algorithm_yaml")
    parser.add_argument("revision", nargs="?")
    parser.add_argument("output_root")
    args = parser.parse_args()

    if args.manifest_algorithms:
        states = read_state_manifest(args.algorithm_yaml)
        print("\n".join(manifest_algorithm_names(states)))
        return

    if args.state_enum_only:
        if args.revision is None:
            parser.error("revision is required for --state-enum-only")
        revision = require_identifier(args.revision, "revision name")
        output_root = Path(args.output_root)
        if not output_root.is_absolute():
            output_root = REPO_ROOT / output_root
        output_root = output_root.resolve()
        if not output_root.is_relative_to(REPO_ROOT):
            raise ValueError("output root must be inside the repository")
        state_enum = (
            output_root / "include" / "sixsim" / "flight" / "profiles"
            / revision / "State.hpp"
        )
        state_enum.parent.mkdir(parents=True, exist_ok=True)
        state_enum.write_text(
            render_state_enum(read_state_manifest(args.algorithm_yaml), revision),
            encoding="utf-8",
        )
        return

    if args.profile_bindings_only:
        if args.revision is None:
            parser.error("revision is required for --profile-bindings-only")
        revision = require_identifier(args.revision, "revision name")
        output_root = Path(args.output_root)
        if not output_root.is_absolute():
            output_root = REPO_ROOT / output_root
        output_root = output_root.resolve()
        if not output_root.is_relative_to(REPO_ROOT):
            raise ValueError("output root must be inside the repository")
        bindings_header = (
            output_root / "include" / "sixsim" / "flight" / "profiles"
            / revision / "AlgorithmBindings.hpp"
        )
        bindings_header.parent.mkdir(parents=True, exist_ok=True)
        bindings_header.write_text(
            render_profile_bindings_header(
                read_state_manifest(args.algorithm_yaml), revision, output_root
            ),
            encoding="utf-8",
        )
        return

    algorithm_path = Path(args.algorithm_yaml).resolve()
    with algorithm_path.open(encoding="utf-8") as stream:
        algorithm = yaml.safe_load(stream)
    if not isinstance(algorithm, dict):
        raise ValueError("algorithm YAML must contain a mapping")
    name = require_identifier(algorithm.get("name"), "algorithm name")
    if algorithm_path.stem != name:
        raise ValueError(
            f"algorithm YAML filename {algorithm_path.stem!r} must match "
            f"its name {name!r}"
        )
    inputs = require_fields(algorithm, "inputs")
    outputs = require_fields(algorithm, "outputs")
    if not inputs:
        raise ValueError("algorithm inputs must not be empty")
    if not outputs:
        raise ValueError("algorithm outputs must not be empty")

    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = REPO_ROOT / output_root
    output_root = output_root.resolve()
    if not output_root.is_relative_to(REPO_ROOT):
        raise ValueError("output root must be inside the repository")

    algorithm_header = (
        output_root / "include" / "sixsim" / "flight" / "algorithms"
        / f"{name}.hpp"
    )
    algorithm_header.parent.mkdir(parents=True, exist_ok=True)
    if not args.mapping_only:
        algorithm_header.write_text(
            render_algorithm_header(name, inputs, outputs), encoding="utf-8"
        )

    if not args.algorithm_header_only:
        if args.revision is None:
            parser.error("revision is required unless --algorithm-header-only is used")
        mapping_header = (
            output_root / "include" / "sixsim" / "flight" / "profiles"
            / args.revision / f"{name}Bindings.hpp"
        )
        mapping_header.parent.mkdir(parents=True, exist_ok=True)
        mapping_header.write_text(
            render_mapping_header(algorithm_path, args.revision, output_root),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
