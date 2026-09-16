# Smoke App

From the repository root, configure and build the smoke scenario.

```sh
./tools/build smoke
```

Run the executable.

```sh
./build/smoke
```

## MATLAB Compare

Build the MATLAB comparison target. CMake generates its `scenario_config.hpp`
from `configs/scenarios/test/matlab_compare.yaml` during the build.

```sh
./tools/build matlab_compare
```

Run the executable.

```sh
./build/matlab_compare
```
