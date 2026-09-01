# Run Logging Design

This note captures the current direction for simulation logging and run
artifacts. It is intentionally narrow; implementation details should be added
only as they are approved in small steps.

## Direction

Simulation logging should start as loop-owned logging. The generated simulation
loop should own the logger and record selected values that are already available
there, such as truth state, model outputs, sensor values, and flight software
outputs.

Model interfaces should remain free of logging parameters for now. A logging
object can be passed into models later if model-private diagnostics become
necessary, but that should be a separate approved change.

The C++ logging API is a small concrete logging sink with named streams and
scalar raw values. Streams can use names such as `truth`, `environment`,
`sensors`, and `flight`. Structured values should be logged as raw scalar
fields, for example `position_ned_m.x`.

Each stream should be written as a wide CSV file. The first row is the header,
with `time_s` as the first column. Each later row is one sample for that stream
at one simulation time.

```csv
time_s,position_ned_m.x,position_ned_m.y,position_ned_m.z
0.0,0.0,0.0,0.0
0.01,0.0,0.0,-0.001
```

Generated simulation code should keep the main loop readable by using small
stream-specific helper functions, such as `log_truth_sample(...)`, instead of
large inline logging calls.

## Run Artifacts

Each simulation run should produce one run output directory. That directory is
the durable record of what was run, what raw data was produced, and what
analysis was generated.

```text
runs/
  <run_name>/
    configs/
      scenario.yaml
    raw/
      truth.csv
      environment.csv
      sensors.csv
      flight.csv
    analysis/
      summary.csv
      plots/
      derived_state.csv
    manifest.yaml
```

The `configs/` directory should contain copies of the configuration files used
for the run. Configs are preserved by copying them into the run directory, not
by writing them through the logging sink. Initially this may only be the
scenario YAML file. Additional config files should be copied only when they are
actually introduced and used by the build or run.

The `raw/` directory should contain the raw logged streams as wide CSV files.

The `analysis/` directory should contain outputs produced by Python analysis
scripts. A future scenario field may identify the analysis script for the run.

The `manifest.yaml` file should index the run artifacts, including the original
scenario path, copied config files, raw stream files, and selected analysis
script or outputs.

## Current Baseline

The current baseline is a concrete C++ `LogSink` that writes wide CSV raw stream
files under `runs/<scenario_name>/raw/`. Generated simulation code logs the
`truth` stream through a small helper function.

The next logging work should stay narrow. Config copying, manifest writing,
additional streams, scenario-controlled run directories, and Python analysis
execution are separate follow-on steps.
