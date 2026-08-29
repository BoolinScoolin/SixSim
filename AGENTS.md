You are working on a new C++ GNC monorepo for a modular six-degree-of-freedom simulation and flight software framework named SixSim.

Your highest-priority requirement is to work in extremely small, reviewable steps.

Do not attempt to build the full architecture, scaffold every directory with implementation files, or anticipate future features. I do not want a large batch of files, utilities, abstractions, tests, or supporting tools from a single agent call.

## Core working rules

1. Plan only one small implementation step at a time.
2. Before writing code, briefly propose the exact scope of that step.
3. Ask for feedback on the proposed scope before implementing it.
4. After approval, implement only that approved scope.
5. Once implemented, stop and propose exactly one small next step.
6. Do not begin the next step without approval.
7. Prefer fewer files, fewer abstractions, and less code.
8. Do not add anything merely because it may be useful later.
9. Do not create placeholder systems, speculative utilities, or broad framework infrastructure.
10. Keep all code readable enough to understand without extensive documentation.
11. Avoid unnecessary inheritance, templates, factories, registries, macros, indirection, or design patterns.
12. Do not create multiple competing implementations unless explicitly requested.
13. Do not refactor unrelated code while completing a step.
14. Do not add optional enhancements after finishing the requested work.
15. When uncertain, choose the smaller implementation and ask.

A suitable step is something as narrow as:

* defining the minimal abstract interfaces for one simulation model category;
* implementing one numerical integrator;
* defining one shared data structure;
* creating one scenario-loading primitive;
* implementing one concrete atmosphere model.

An unsuitable step would be:

* defining every simulation model interface at once;
* building all model factories;
* creating all application entry points;
* implementing SITL, HIL, replay, and Monte Carlo together;
* generating dozens of files;
* adding tests, documentation, CI, formatting, logging, validation, and build tooling in one pass.

I never want to receive two dozen new files or tools to review from one agent call.

## Development sequence

Work from fundamental software abstractions outward.

A likely early step may be defining the minimal simulation model abstract base classes, but do not assume their interfaces without first proposing them for review.

Keep each proposal low-level and concrete. For example:

> Proposed step: define only the atmosphere model interface and its output structure. No concrete atmosphere implementation, factory, scenario parsing, or tests yet.

That is an appropriate scope.

This is not appropriate:

> Proposed step: define all model interfaces, implement factories, add ISA atmosphere, constant wind, sensor models, actuator models, scenario loading, tests, and CMake integration.

## Testing policy

Do not use test-driven development.

Do not write tests alongside each implementation step unless I explicitly request them.

Tests will be added after the core architecture and implementations are stable enough to review. I do not want to review speculative tests while the interfaces are still changing.

When tests are eventually added, the intended coverage is:

* unit tests for simulation models;
* unit tests for numerical integrators and other fundamental algorithms;
* unit tests for flight software algorithms;
* simulation integration tests against known scenarios;
* closed-loop integration tests only after simulation and flight software are mature.

Do not create empty test files or test scaffolding preemptively.

## Hardware policy

Do not implement hardware-specific code until the end.

This includes:

* Teensy drivers;
* physical sensor drivers;
* board pin mappings;
* I2C, SPI, UART, PWM, and storage implementations;
* hardware-specific HAL implementations;
* HIL-specific device communication.

The initial focus is platform-independent simulation and flight software architecture.

The HAL may be defined when needed, but physical hardware implementations should wait until the software-facing interfaces are stable.

## Architectural direction

Current repository structure:

```text
SixSim/
  firmware/
    platformio.ini
    src/
      main_teensy.cpp

  flight/
    include/
    src/

    tests/
      unit/
      integration/

  hal/
    include/
    src/
      teensy/
      sitl/
      replay/
      hil/

    tests/
      unit/
      integration/

  sim/
    include/
    src/

    models/
      dynamics/
      aero/
      atmosphere/
      wind/
      sensors/
      actuators/

    tests/
      unit/
        models/
          dynamics/
          aero/
          atmosphere/
          wind/
          sensors/
          actuators/
        integrators/
        math/

      integration/
        scenario_loading/
        model_composition/
        closed_loop/

      regression/
        reference_cases/
        baselines/

  apps/
    sitl/
    replay/
    monte_carlo/
    hil/

  configs/
    scenarios/
      test/
    monte_carlo/

  tools/
    config_validator/
    codegen/
    plotting/
    log_tools/

  generated/
    include/
    src/
```

This structure is directional, not permission to populate every directory.

Leave directories empty until actual functionality requires files inside them.

In particular, keep `flight/src/` empty until a specific flight algorithm is intentionally introduced.

## Simulation model philosophy

Simulation model directories contain actual implementations.

For example:

```text
sim/models/atmosphere/
```

may eventually contain implementations such as:

```text
isa1976.cpp
constant_atmosphere.cpp
tabulated_atmosphere.cpp
```

The differences between atmosphere models may be structural, not merely different parameter values. Do not force every implementation into an overly restrictive configuration schema.

A scenario selects the implementation to use and provides whatever parameters that implementation requires.

There is no separate `sim/configs/` directory.

## Configuration philosophy

Keep centralized configuration minimal.

For now:

```text
configs/
  scenarios/
  monte_carlo/
```

A scenario should eventually contain everything needed to construct and execute one simulation run, including selections for:

* dynamics;
* aerodynamics;
* atmosphere;
* wind;
* sensors;
* actuators;
* numerical integration;
* initial conditions;
* simulation timing;
* flight software participation;
* logging.

Do not split vehicle, environment, mission, or simulated hardware configuration into separate top-level directories unless repeated use creates a clear need later.

Do not build scenario parsing until it becomes the next approved step.

Monte Carlo configuration should remain separate because it describes distributions, dispersions, run counts, seeds, and campaign-level behavior rather than a single nominal run.

## Flight software philosophy

Flight software algorithms should operate on shared input and output data structures.

Flight algorithms should not directly call sensor or hardware interfaces.

The runtime or HAL is responsible for moving physical or simulated data into shared structures and moving outputs back toward simulated or physical actuators.

Conceptually:

```text
physical or simulated source
          |
          v
         HAL
          |
          v
shared flight data structures
          |
          v
flight algorithms
```

Do not introduce flight software object hierarchies, direct sensor ownership, or hardware coupling without explicit approval.

Configuration associated with a future flight algorithm may live beside its `.cpp` file, but only when that algorithm actually exists and needs configuration.

Do not pre-create flight modules or flight YAML files.

## HAL philosophy

The HAL should move data between runtime-specific sources and the shared flight data structures.

Flight algorithms should not include files such as:

```cpp
#include "hal/imu.hpp"
#include "hal/clock.hpp"
```

unless the architecture is explicitly changed later.

The likely pattern is closer to:

```cpp
hal.read_inputs(shared_data);
flight.step(shared_data);
hal.write_outputs(shared_data);
```

Do not implement the complete HAL now. Define only the minimum portion required by an approved step.

## Code quality

Prefer direct, ordinary C++.

Use:

* clear names;
* small interfaces;
* straightforward ownership;
* minimal public APIs;
* explicit data flow;
* comments only where the intent is not obvious.

Avoid:

* clever abstractions;
* generic frameworks;
* premature extensibility;
* deep inheritance trees;
* excessive file splitting;
* wrapper classes with no meaningful behavior;
* one class per trivial concept;
* configuration systems before configuration is needed;
* generated code before generated code solves a real problem.

A single readable file is often preferable to several highly abstract files.

## Response format for every step

Before implementation, respond with:

### Proposed step

A one- or two-sentence description of the exact change.

### Files affected

List only the files that would be created or modified.

### Explicitly excluded

State the nearby work that will not be included.

### Design questions

Ask only questions that materially affect this immediate step.

Do not implement anything until I approve the scope.

After implementation, respond with:

### Completed

Briefly state what changed.

### Files changed

List the files actually created or modified.

### Review focus

Point me to the small number of decisions or code sections I should inspect.

### Proposed next step

Propose exactly one narrowly scoped next step.

Do not implement that next step.

## First task

Inspect the current repository and propose the smallest reasonable first architectural step.

A likely candidate is one minimal simulation model interface, but do not assume that is correct until you inspect the repository.

Do not write code yet.
