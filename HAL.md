# SixSim HAL Direction

## Purpose and implementation policy

This document is the source of architectural direction for backend HALs, SITL
devices, and their connection to simulation and flight software. The working rules in
[AGENTS.md](AGENTS.md) still apply: propose one small step, obtain approval,
implement only that step, and stop. This roadmap is not permission to implement
all steps at once.

The first milestone is configurable ideal altimeters in separate single-vehicle
simulation runs. Simultaneous vehicles and multiple flight computers are future
capabilities, not part of this milestone.

## Current foundation

Already implemented:

- `hal/include/sixsim/hal/altimeter.hpp`: `Altimeter` and `AltimeterSample`, with
  MSL altitude, measurement time since runtime start, and validity.
- `hal/sitl/altimeter.hpp`: `SitlAltimeter` computes MSL altitude from simulation
  truth, buffers the sample, and exposes it through the `Altimeter` interface.
  It starts invalid; `read()` returns the latest sample without waiting.
- `Environment::origin_altitude_msl_m` and optional scenario YAML/codegen support,
  defaulting to zero when omitted.
- `Simulation` directly owns a configured vehicle vector, and each
  `Vehicle<Fcus>` owns its generated tuple of concrete FCUs. Generated
  `build_vehicle()` constructs the selected FCU revisions and their configured
  SITL sensors.
- `SensorTruthInputs`, `SitlSensor`, `SitlSensors`, and `SitlDeviceRegistry`
  provide the current sensor update and ownership path.
- Generated profile bindings expose the selected FCU's fixed SITL device tree,
  and `dummy_uno_r3::SitlHal` owns a stable registry, its generated binding, and its
  cycle counter.

Multiple-vehicle stepping, actuator support, and a complete physical sensor suite
remain future work.

## Ownership and dependencies

Use these locations as functionality is introduced:

```text
hal/
  include/sixsim/hal/    # Platform-independent interfaces
  physical/<fcu_name>/   # Physical implementations grouped by FCU
  sitl/                  # Simulation implementations
```

Organize physical implementations by FCU rather than microcontroller family.
Different FCUs may use arbitrary microcontrollers, including the same MCU in
otherwise different hardware. Do not populate directories preemptively.

Each FCU revision has one shared profile-level FCU parameterized by a backend HAL,
for example `dummy_uno_r3::Fcu<Hal>`. The FCU owns scheduling and flight state. The
backend HAL owns or binds every environment-facing operation, including the cycle
tick source, device access, and platform initialization. Physical and SITL
specializations therefore share FCU behavior without sharing implementation:

```cpp
namespace sixsim::hal::dummy_uno_r3 {
using PhysicalFcu = Fcu<PhysicalHal>;
using SitlFcu = Fcu<SitlHal>;
}
```

The backend HAL contract is structural C++17, not a universal abstract base
class. At minimum, a backend supplies `cycle_ticks()` and `devices()`. A SITL
backend may additionally expose simulation-only update operations; physical
firmware must not depend on them.

Use `Physical` and `Sitl` to distinguish backend implementations. Use `Devices`
for the flight-facing collection instead of using `Hardware` for both physical
and simulated objects. For example, `dummy_uno_r3::PhysicalHal` exposes
`PhysicalDummyUnoR3Devices`, while `dummy_uno_r3::SitlHal` exposes
`generated::dummy_uno_r3::SitlDevices`. Flight code reaches the common structural
surface through `fcu.devices()` and continues to depend on narrow device
interfaces rather than a universal device inventory.

The intended SITL ownership is:

```text
Simulation
  vehicles_: vector<Vehicle>
    Vehicle
      name
      rigid-body state, mass properties, and actuator state
      aerodynamic and propulsion models
      generated tuple of concrete FCUs
        dummy_uno_r3::Fcu<dummy_uno_r3::SitlHal>
          flight scheduler and flight state
          dummy_uno_r3::SitlHal
            cycle tick counter
            generated::dummy_uno_r3::SitlDevices (typed flight bindings)
            unique_ptr<SitlDeviceRegistry> (runtime ownership and updates)
              SitlSensors
                vector<unique_ptr<SitlSensor>>
                  SitlAltimeter, SitlTimer, SitlImu, ...
              SitlActuators (introduced when an actuator is needed)
```

Sensors are read-only inputs available to flight software, including clocks as
well as physical measurement transducers. Each concrete SITL sensor is one object
that implements its device-specific HAL interface and a common update operation.
Physical measurement sensors own their mounting configuration, measurement
effects, buffered sample, and logic for computing their ideal response from
simulation truth. `SitlSensors` owns the heterogeneous collection through
`std::unique_ptr<SitlSensor>` and updates every object through that interface.
Adding a second sensor creates a second instance, not a new field in a universal
measurement structure. Do not introduce a general registry or runtime factory
framework for this milestone.

Assembly retains typed references when adding concrete objects so flight wiring
does not repeatedly search or downcast the collection. A generic `SitlSensor`
reference is for ownership and updating; flight reads through typed device
interfaces such as `Imu&` and `Altimeter&`.

Each FCU revision profile defines a fixed flight-facing device layout with nested
sensor and, when needed, actuator groups. Profile device keys become getter names,
so a `system_timer` key produces
`fcu.devices().sensors().system_timer()`. Code generation produces only the
binding and access surface required by the selected profile, not one class per
device. Add groups and getters only for devices that exist in that profile.

The validated YAML profile name is used directly as its C++ namespace and path
segment. Fixed role names such as `Fcu`, `PhysicalHal`, `PhysicalFcu`, `SitlHal`,
and `SitlFcu` live inside that namespace. Code generation therefore derives
profile include paths and concrete types without a separately maintained C++
type catalog.

Physical and SITL HAL implementations of the same FCU revision present this same
getter structure even though their ownership differs. The physical HAL owns its
fixed drivers. The SITL HAL owns its `SitlDeviceRegistry` at a stable address,
resolves profile names and HAL types once during initialization, and retains
typed references in the generated profile binding. These references are a live
interface to buffered devices, not a per-cycle snapshot.

There is no generic SITL FCU abstraction. Profile-specific SITL HALs own their
runtime registry and tick state, while generated bindings provide their exact
flight-facing getters. Do not put every profile's getters on generic
`SitlSensors`.

Scenario generation supplies the concrete FCU storage type. For the current
single-FCU scenario it is equivalent to:

```cpp
using ConfiguredFcus = std::tuple<hal::dummy_uno_r3::SitlFcu>;
using ConfiguredVehicle = sim::Vehicle<ConfiguredFcus>;
```

`Vehicle<Fcus>` owns that tuple by value. Generic simulation code visits it with
`std::apply` and a templated operation, so each concrete FCU retains its exact C++
type:

```cpp
std::apply(
    [&](auto&... fcu) {
      (update_fcu(fcu, inputs, dt_s), ...);
    },
    vehicle.fcus);
```

`update_fcu` calls `fcu.hal().update_sensors(inputs)`, checks `fcu.check_cycle()`
against the tick snapshot for the current simulation time, and invokes the
shared `run_cycle(fcu)` when the scheduler reports a due cycle. It then advances
ticks for the next simulation step. This is compile-time iteration, not a
universal FCU base class, runtime registry lookup, variant, or type-erased
callback. Repeated profile types remain distinct tuple elements, so a vehicle
may contain two FCUs of the same revision with independent state.

The generated scenario header provides `ConfiguredFcus`, `ConfiguredVehicle`,
and `build_vehicle()`. Scenario-compiled simulation sources use that configured
vehicle type; model libraries and public HAL interfaces do not include scenario
output. A scenario with no FCUs uses `std::tuple<>`, and the same fold performs
no work.

Flight-cycle composition uses one shared C++17 `run_cycle(Fcu&)` function
template. Physical and SITL specializations of the profile FCU satisfy the same
structural API. Scheduling remains outside the cycle logic. Do not introduce a
universal HAL base class, perform registry lookup or downcasting per cycle, or
duplicate the flight-cycle implementation by backend.

`Vehicle<Fcus>` represents the whole simulated vehicle and owns its FCU tuple
directly; do not add a `VehicleInstance` wrapper. Vehicle names identify vehicles
within a simulation; FCU names identify computers within their owning vehicle.
Every FCU belongs to exactly one vehicle. `Simulation` schedules execution while
ownership keeps the vehicle association explicit, without a separate
FCU-to-vehicle lookup.

This tree is the target ownership, not the current implementation. Introduce the
configured vehicle type and tuple iteration in separately approved steps. Do not
introduce a temporary `Simulation::devices_` member or a second generic FCU
wrapper during the transition.

Public HAL interfaces must not depend on simulation types, Arduino, PlatformIO,
or FCU-specific drivers. SITL sensor implementations may depend on simulation
types. Embedded firmware must not depend on simulation components.

## Measurement and control data flow

```text
Vehicle truth + sensor placement + physical motion
                       |
                       v
Concrete SITL sensor computes ideal per-instance reading
                       |
                       v
Same sensor applies effects and buffers its HAL sample
                       |
                       v
Flight snapshots samples, runs algorithms, writes HAL commands
                       |
                       v
SITL actuators apply commands to simulated actuator state
```

Each concrete SITL sensor calculates its physical response at its mounting
location. Future IMU instances account for mounting orientation, rotational
motion, and flexible effects. The same sensor then applies measurement behavior
such as noise, bias, quantization, saturation, sampling, and latency. None of
these additional effects is included in the first ideal-altimeter milestone.

A small simulation-truth view supplies current simulation time, rigid-body state,
and environment by const reference to sensors during an update. It is a view of
existing physical data, not an aggregate inventory of readings. Sensors must not
retain the view beyond the update. Extend it only when an implemented sensor
needs additional dynamics results; do not pass the whole `Simulation` object, add
speculative IMU fields, or recompute vehicle dynamics in HAL.

The ideal altimeter calculation is:

```cpp
altitude_msl_m = origin_altitude_msl_m - position_ned_m.z;
```

Its timestamp is the current simulation time. Sensor model logic is handwritten,
not generated and not part of the public HAL interface.

Flight cycle code may call small HAL interfaces directly. Capture readings once
at the beginning of each control cycle, pass the fixed samples to estimation and
control algorithms, then write resulting commands through HAL. Algorithms consume
data rather than reading sensors themselves. No giant flight I/O structure or
separate coordinator class is required.

Sensor updates happen before flight reads. Actuator updates happen after flight
writes. Keep those calls separate rather than combining them into one hardware
update. Measurement timestamps are preserved: a cycle snapshot does not imply
that all sensors measured at the same instant.

## Single-FCU cycle timing

The FCU uses one fixed cycle period. In SITL, that period must be an integer
multiple of the fixed simulation step size:

```text
fcu_period_s = N * simulation_dt_s, where N is an integer >= 1
```

Reject incompatible combinations rather than rounding the period or shortening
physics steps. For example, a 100 Hz FCU has a 10 ms period and executes every
10 steps with a 1 ms simulation step; a 3 ms simulation step is incompatible.

SITL executes the first flight cycle at simulation time zero, after sensor
updates, and then every N simulation steps. Use an integer step counter to
select cycles. The rate is measured in simulation time, independent of how fast
the host executes the simulation.

Each FCU profile declares its logical base tick rate. In the current profiles it
is 1 kHz, so one tick represents one millisecond. In SITL, the counter starts at
zero and advances by the number of base ticks represented by each physics step.
After updating sensors at the matching simulation time, the simulation passes a
tick snapshot to the flight scheduler, including at time zero. In hardware, an
interval timer interrupt increments the counter once per base tick; the main
loop reads a safe snapshot and passes it to the flight scheduler. Flight software
does not execute in the interrupt. A platform without an interval timer may
derive the counter from elapsed clock time. Counter access must be safe for the
target platform.

The profile FCU reads this snapshot through its backend HAL's `cycle_ticks()`
operation. This scheduler clock is runtime machinery and is distinct from a
flight-visible `Timer` device. Timers remain named sensors under
`fcu.devices().sensors()` and may expose elapsed time to flight software; they do
not drive the scheduler directly.

Flight software owns the rate division. Each FCU profile declares an integer
cycle rate, represented by a whole-tick period on that FCU's configured base
clock.
SITL additionally requires the simulation step to divide that period exactly.
For example, at a 1 kHz base rate, a 100 Hz FCU has a 10 ms period and accepts
1 ms, 2 ms, or 5 ms simulation steps, but rejects a 3 ms step. Hardware follows
the same FCU period but has no physics-step constraint. The backend does not
maintain a separate flight-release counter.

The flight scheduler derives the current cycle number by integer division of the
tick snapshot by the ticks per flight cycle, and retains the last-handled cycle
number.
Execute the initial cycle at tick zero once. Thereafter, an unchanged cycle
number does no flight work; an advanced cycle number executes one flight cycle
and logs the skipped count (elapsed cycles minus one). Mark only the cycle
derived from the supplied snapshot as handled, so ticks arriving during flight
execution remain available to the next update. Never replay skipped cycles in
a catch-up loop. Counter wraparound must preserve this behavior when implemented.

A late cycle runs as soon as execution resumes. Subsequent releases retain their
original phase; do not restart the interval from the late execution. For example,
at 100 Hz, resuming at 37 ms after the last handled release at time zero executes
once for the 30 ms release, records two skipped cycles and 7 ms lateness, and
leaves the next release at 40 ms. That next cycle can start on time if the late
cycle finishes before then. A modulo-only check would miss the late cycle at
37 ms; cycle-number comparison detects it. The tick snapshot and tick period
provide lateness at tick resolution; finer measurement requires the actual start
time as well as the scheduled cycle time.

This section defines behavior only; counter implementation, interrupt wiring,
and logging implementation remain separate steps.

## Multiple FCUs

A vehicle may own multiple independent FCUs. Each FCU belongs to that vehicle
and consumes its truth while owning separate hardware and flight state:

```text
Vehicle "booster"
  physical state and models
  FCU "dummy_uno_r3": generated devices + typed flight instance
  FCU "dummy_fcu_rev2": generated devices + typed flight instance
```

Keep sensor names local to each hardware instance. Use explicit flight-to-HAL
bindings and no global mutable sensor or flight instances. Hardware instances
must retain independent samples, measurement effects, timing, and commands.

Each FCU is identified by a hardware revision, such as `dummy_uno_r3`, rather
than by its execution backend. An explicit multi-FCU configuration schema remains
a separate design step. Each FCU's sensors use the physical truth of its owning
vehicle; multiple FCUs do not require changes to individual device HAL interfaces.

Multi-FCU scheduling, shared physical sensor connections, and actuator command
authority require explicit future designs. Never silently use last-write-wins
behavior when two FCUs command one physical actuator.

## Configuration and minimal code generation

Extend the existing scenario generator; do not build a separate configuration
framework. A scenario selects a reusable FCU revision profile:

```yaml
vehicle:
  fcus:
    - dummy_uno_r3
```

The list contains profile references, not execution-backend names. A single
entry is the normal initial case; additional entries represent additional FCUs
owned by the same vehicle.

The profile is stored separately and is reusable across scenarios:

```text
configs/
  fcus/
    dummy_uno_r3.yaml
```

The profile device tree mirrors the flight-facing access structure:

```yaml
devices:
  serial:
    type: serial
    baud_rate: 115200
  sensors:
    altimeter:
      type: altimeter
    timer:
      type: timer
```

The profile owns the revision's device suite as those interfaces are introduced.
`dummy_uno_r3.yaml` currently declares serial configuration and named altimeter
and timer sensors.
Each device key is both its profile-local identity and the source of its generated
flight-facing getter name. Prefer names that describe hardware roles, such as
`system_timer`, when multiple devices implement the same HAL interface. Do not
add backend or MCU fields until the corresponding interfaces exist. SITL is the
execution backend, not part of the FCU identity. Physical implementations use
the same revision identity under `hal/physical/dummy_uno_r3/`.

Generated vehicle construction resolves selected FCU profiles and emits only the
construction and initialization bindings needed for their configured instances.
It also emits a tuple containing the explicitly named handwritten FCU type for
each configured entry, preserving YAML order. Build metadata selects only the
headers and generated profile bindings required by those tuple elements.
Physical construction binds fixed implementations; SITL initialization creates
and populates one owned `SitlDeviceRegistry` per FCU, then constructs the matching
profile HAL and FCU. Generated code contains no sensor equations or update loops.
Two configurations may select different inventories without changing handwritten
sensor behavior.

## Separately approved implementation sequence

Each numbered item is a separate proposal and approval boundary. Before editing
code, identify its exact files and interfaces; after implementing it, stop.

1. **Completed:** Define the minimal truth view and common `SitlSensor` update interface. Extend
   `SitlAltimeter` so one object computes ideal MSL altitude from origin altitude
   minus NED Z, then applies its current buffering behavior.
2. **Completed:** Implement `SitlSensors` ownership of a heterogeneous sensor collection, named
   concrete construction, stable instances, typed HAL references, and one update
   loop.
3. **Completed:** Add the device owner now named `SitlDeviceRegistry`, containing
   `SitlSensors`. Do not create an empty actuator subsystem; retain
   `SitlActuators` as the intended ownership when needed.
4. **Completed and superseded:** Introduce one minimal vehicle-owned transitional
   FCU owning the device collection. The generated concrete-FCU tuple path later
   replaced and removed that transitional type.
5. **Completed:** Wire updates through the vehicle's FCU and add necessary host include paths.
   Update sensors from matching time and owning vehicle state before event
   evaluation, including initial and terminal states. Initially sensors may be empty.
6. **Completed:** Define the FCU profile location and collection-based reference syntax. Add
   minimal generator resolution for one selected profile; keep its schema limited
   to implemented hardware.
7. **Completed:** Create `configs/scenarios/test/hal_altimeter.yaml`, based on the existing smoke
   scenario, selecting `dummy_uno_r3` and exercising its named altimeter.
8. **Completed:** Add a dedicated `hal_altimeter` host target using that scenario and the
   existing runtime entry point.
9. **Completed:** Move the existing physical device ownership and cycle tick access behind
   `dummy_uno_r3::PhysicalHal`. Parameterize the handwritten `dummy_uno_r3::Fcu`
   on its backend HAL and expose `dummy_uno_r3::PhysicalFcu` as the firmware
   specialization.
   Do not add the SITL backend or profile binding generation in this step.

Do not add HAL functionality to `matlab_compare.yaml` or change its target's
scenario selection. Keep it aligned with the MATLAB implementation. The new HAL
scenario is a manual integration exercise, not an automated assertion suite.

Defer noise, IMUs, mounting configuration, flight algorithms, physical drivers,
simultaneous vehicles, and multi-FCU execution. Do not add automated tests without
separate approval.

## Verification and milestone acceptance

During implementation, use focused compile/build checks and manual inspection.
The future milestone is complete when:

- Existing scenarios without sensors remain supported.
- Zero, one, and two altimeters assemble as independent configured instances.
- Generated scenario C++ contains assembly only; generated profile accessors
  contain no device behavior.
- A 200 m MSL origin and NED Z of -50 m produce 250 m MSL with the matching
  simulation timestamp.
- Samples begin invalid and become valid after an update. Repeated reads retain
  the buffered value and timestamp until the next update.
- Existing host targets and the new `hal_altimeter` target build.

These are acceptance criteria, not claims that the milestone is already complete.

## Firmware background

PlatformIO remains the embedded build and deployment layer, not a second flight
software codebase. Do not copy shared SixSim sources into firmware. Keep board
APIs below HAL and firmware entry points small. Additional physical drivers
remain deferred until their software-facing interfaces are stable.

The existing dummy Uno R3 project is under `firmware/dummy_uno_r3`. From the
repository root:

```bash
pio run --project-dir firmware/dummy_uno_r3 -e dummy_uno_r3
pio run --project-dir firmware/dummy_uno_r3 -e dummy_uno_r3 --target upload
```

Generated `.pio/` artifacts stay out of Git. Preserve CMake host workflows and do
not assume every flight algorithm will fit a resource-constrained Uno R3.
