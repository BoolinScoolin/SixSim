cmake_minimum_required(VERSION 3.20)

project(SixSim LANGUAGES CXX)

add_executable(rigid_body_smoke
  apps/smoke/rigid_body_smoke.cpp
  sim/src/run_simulation.cpp
  sim/src/simulation_event.cpp
  sim/models/aerodynamics/zero_aerodynamics.cpp
  sim/models/atmosphere/constant_atmosphere.cpp
  sim/models/gravity/constant_gravity.cpp
  sim/models/propulsion/zero_propulsion.cpp
  sim/models/wind/constant_wind.cpp
  sim/models/dynamics/rigid_body_dynamics.cpp
)

target_compile_features(rigid_body_smoke PRIVATE cxx_std_20)

target_include_directories(rigid_body_smoke PRIVATE
  .
  include
  sim/include
)
