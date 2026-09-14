# Smoke App

Configure and build the smoke executable.

```sh
cmake -S . -B build
cmake --build build
```

Run the executable.

```sh
.\build\Debug\rigid_body_smoke.exe
```

## MATLAB Compare

Build the MATLAB comparison target. CMake generates its `scenario_config.hpp`
from `configs/scenarios/test/matlab_compare.yaml` during the build.

```sh
cmake --build build --target matlab_compare
```

Run the executable.

```sh
.\build\Debug\matlab_compare.exe
```
