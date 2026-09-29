from pathlib import Path
import subprocess

Import("env")

repo_root = Path(env["PROJECT_DIR"]).resolve().parents[1]
generator_path = repo_root / "tools" / "codegen" / "generate_fcu_config.py"
profile_name = env.GetProjectOption("board_build.fcu_profile")
generated_directory = Path(env.subst("$BUILD_DIR")) / "generated"
generated_header = generated_directory / f"{profile_name}_config.hpp"
common_values_header = (
    repo_root
    / "flight"
    / "src"
    / "profiles"
    / profile_name
    / "common_values.hpp"
)
state_manifest = (
    repo_root
    / "flight"
    / "src"
    / "profiles"
    / profile_name
    / "state_machine.yaml"
)

if common_values_header.is_file():
    algorithm_generator = (
        repo_root / "flight" / "codegen" / "generate_algorithm_artifacts.py"
    )
    algorithms_directory = repo_root / "flight" / "src" / "algorithms"
    generated_flight_root = repo_root / "flight" / "build" / "generated"
    if not state_manifest.is_file():
        raise RuntimeError(
            f"FCU profile {profile_name} has no state/algorithm manifest: "
            f"{state_manifest}"
        )

    subprocess.check_call(
        [
            "python3",
            str(algorithm_generator),
            "--state-enum-only",
            str(state_manifest),
            profile_name,
            str(generated_flight_root),
        ]
    )
    algorithm_names = subprocess.check_output(
        [
            "python3",
            str(algorithm_generator),
            "--manifest-algorithms",
            str(state_manifest),
            "unused_revision",
            "unused_output_root",
        ],
        text=True,
    ).splitlines()
    for algorithm_name in algorithm_names:
        algorithm_yaml = algorithms_directory / f"{algorithm_name}.yaml"
        subprocess.check_call(
            [
                "python3",
                str(algorithm_generator),
                str(algorithm_yaml),
                profile_name,
                str(generated_flight_root),
            ]
        )
    subprocess.check_call(
        [
            "python3",
            str(algorithm_generator),
            "--profile-bindings-only",
            str(state_manifest),
            profile_name,
            str(generated_flight_root),
        ]
    )

    env.BuildSources(
        str(Path(env.subst("$BUILD_DIR")) / "flight_algorithms"),
        str(algorithms_directory),
        src_filter=[f"+<{algorithm_name}.cpp>" for algorithm_name in algorithm_names]
        + ["-<*>"],
    )

subprocess.check_call(
    [
        "python3",
        str(generator_path),
        profile_name,
        str(generated_header),
    ]
)

env.Append(
    CPPPATH=[
        str(generated_directory),
        str(repo_root / "include"),
    ]
)
