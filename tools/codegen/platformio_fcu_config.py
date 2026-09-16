from pathlib import Path
import subprocess

Import("env")

repo_root = Path(env["PROJECT_DIR"]).resolve().parents[1]
generator_path = repo_root / "tools" / "codegen" / "generate_fcu_config.py"
profile_name = env.GetProjectOption("board_build.fcu_profile")
generated_directory = Path(env.subst("$BUILD_DIR")) / "generated"
generated_header = generated_directory / f"{profile_name}_config.hpp"

subprocess.check_call(
    [
        "python3",
        str(generator_path),
        profile_name,
        str(generated_header),
    ]
)

env.Append(CPPPATH=[str(generated_directory)])
