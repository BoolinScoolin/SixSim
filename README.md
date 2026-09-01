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

## Running Simulations

Run the MATLAB comparison scenario from the repository root with:

```sh
./build/matlab_compare_smoke
```

By default, raw logs are written under `runs/matlab_compare/`. Use `--output`
to provide a complete run directory instead:

```sh
./build/matlab_compare_smoke --output runs/my_matlab_run
```

## YAML Code Generation

`run_simulation.cpp` files are generated from YAML scenarios during the CMake
build. Generated C++ under `build/generated/` is not committed.

Each scenario must specify a positive logging rate under `simulation`:

```yaml
simulation:
  logging_rate_hz: 100.0
```

Samples are written on simulation-step boundaries. When a requested sample time
falls between steps, it is written on the first step at or after that time.

The generator keeps model and integrator support organized by category. To
register a new supported type, add its parser, C++ include renderer, C++
construction or advance-state renderer, and table entry in that category's
section of `tools/codegen/generate_run_simulation.py`.
