# SixSim

SixSim is a C++ six-degree-of-freedom simulation and flight software framework.
The repository is currently focused on platform-independent simulation pieces
and a small smoke executable.

## Dependencies

The CMake build uses Python for YAML-based code generation. Install the Python
codegen dependencies once into the Python environment you use for CMake:

```sh
python -m pip install -r tools/codegen/requirements.txt
```

On Windows PowerShell, a local virtual environment keeps these dependencies
isolated:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/codegen/requirements.txt
cmake -S . -B build -DPython3_EXECUTABLE="$PWD\.venv\Scripts\python.exe"
```

CMake checks for these dependencies during configure and prints the install
command if they are missing.

## CMake

Configure and build with CMake:

```sh
cmake -S . -B build
cmake --build build
```

The current smoke executable is `rigid_body_smoke`.

## YAML Code Generation

`run_simulation.cpp` files are generated from YAML scenarios during the CMake
build. Generated C++ under `build/generated/` is not committed.

The generator keeps model and integrator support organized by category. To
register a new supported type, add its parser, C++ include renderer, C++
construction or advance-state renderer, and table entry in that category's
section of `tools/codegen/generate_run_simulation.py`.
