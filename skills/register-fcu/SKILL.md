---
name: register-fcu
description: Register a new FCU revision in SixSim by adding its profile and HAL backend files, following existing profiles while keeping the change specific to the hardware.
---

# Register an FCU revision

Use this skill when adding a new FCU revision to SixSim. It is intentionally rough:
keep it current as the registration process is exercised, and prefer checking the
current code over assuming this outline covers every case.

## Initial outline

For a new revision, identify its revision name and inspect a similar existing
profile, especially `dummy_uno_r3`. The initial registration areas are:

1. Add `configs/fcus/<revision>.yaml` with the revision's supported devices and
   timing. Include only hardware that has an implemented HAL interface.
2. Add `hal/include/sixsim/hal/profiles/<revision>.hpp` for the revision's FCU
   type and its backend-independent flight computer behavior.
3. Add `hal/sitl/<revision>/hal.hpp` to connect the profile to its SITL devices
   and simulation-owned device registry, when SITL support is part of the task.
4. Add `hal/physical/<revision>/` for the physical HAL and device ownership.
   Physical drivers are hardware-specific and must use the `Physical` prefix in
   concrete implementation types.

The profile name is used as the C++ namespace and path segment. Concrete backend
types use `Sitl` or `Physical` prefixes; shared profile types remain backend
independent. Refer to `HAL.md` for ownership and sensor data-flow direction.

## While registering

- Work in small, reviewed steps. Confirm the exact file scope before editing.
- Treat the YAML device list as the revision's profile, not as a place to add
  devices that have no corresponding interface or implementation.
- Inspect how the profile is consumed by the existing generators and firmware
  target; don't add generator or build changes unless registration requires them.
- Update this skill with concrete discoveries from the registration, keeping the
  procedure concise and specific to what the repository actually does.
