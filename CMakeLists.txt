cmake_minimum_required(VERSION 3.20)

project(SixSim LANGUAGES CXX)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SIXSIM_CODEGEN_PYTHONPATH "$ENV{PYTHONPATH}")

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import yaml"
  RESULT_VARIABLE SIXSIM_CODEGEN_IMPORT_RESULT
  OUTPUT_QUIET
  ERROR_QUIET
)

if(NOT SIXSIM_CODEGEN_IMPORT_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Python code generation dependencies are missing. "
    "Install them into the active Python environment with: "
    "${Python3_EXECUTABLE} -m pip install -r ${CMAKE_SOURCE_DIR}/tools/codegen/requirements.txt"
  )
endif()

function(add_generated_simulation output scenario)
  add_custom_command(
    OUTPUT ${output}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/generated
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tools/codegen/generate_run_simulation.py
            ${scenario}
            ${output}
    DEPENDS
      ${scenario}
      ${CMAKE_SOURCE_DIR}/tools/codegen/generate_run_simulation.py
      ${CMAKE_SOURCE_DIR}/tools/python/sixsim_gnc/attitude.py
      ${CMAKE_SOURCE_DIR}/tools/python/sixsim_gnc/__init__.py
    VERBATIM
  )
endfunction()

set(SIXSIM_SMOKE_RUN_SIMULATION ${CMAKE_BINARY_DIR}/generated/run_simulation.cpp)
set(SIXSIM_MATLAB_COMPARE_RUN_SIMULATION ${CMAKE_BINARY_DIR}/generated/matlab_compare_run_simulation.cpp)

add_generated_simulation(
  ${SIXSIM_SMOKE_RUN_SIMULATION}
  ${CMAKE_SOURCE_DIR}/configs/scenarios/test/smoke.yaml
)

add_generated_simulation(
  ${SIXSIM_MATLAB_COMPARE_RUN_SIMULATION}
  ${CMAKE_SOURCE_DIR}/configs/scenarios/test/matlab_compare.yaml
)

add_executable(rigid_body_smoke
  apps/smoke/rigid_body_smoke.cpp
  ${SIXSIM_SMOKE_RUN_SIMULATION}
  sim/src/logging.cpp
  sim/models/aerodynamics/simple_aerodynamics_model_v1.cpp
  sim/models/aerodynamics/zero_aerodynamics.cpp
  sim/models/atmosphere/constant_atmosphere.cpp
  sim/models/gravity/constant_gravity.cpp
  sim/models/propulsion/hardcoded_thrust_curve_propulsion.cpp
  sim/models/propulsion/zero_propulsion.cpp
  sim/models/wind/constant_wind.cpp
)

target_compile_features(rigid_body_smoke PRIVATE cxx_std_20)

target_include_directories(rigid_body_smoke PRIVATE
  .
  include
  sim/include
)

add_executable(matlab_compare_smoke
  apps/smoke/rigid_body_smoke.cpp
  ${SIXSIM_MATLAB_COMPARE_RUN_SIMULATION}
  sim/src/logging.cpp
  sim/models/aerodynamics/simple_aerodynamics_model_v1.cpp
  sim/models/aerodynamics/zero_aerodynamics.cpp
  sim/models/atmosphere/constant_atmosphere.cpp
  sim/models/gravity/constant_gravity.cpp
  sim/models/propulsion/hardcoded_thrust_curve_propulsion.cpp
  sim/models/propulsion/zero_propulsion.cpp
  sim/models/wind/constant_wind.cpp
)

target_compile_features(matlab_compare_smoke PRIVATE cxx_std_20)

target_include_directories(matlab_compare_smoke PRIVATE
  .
  include
  sim/include
)
