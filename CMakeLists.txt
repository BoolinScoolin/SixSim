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

function(add_generated_scenario config_header scenario)
  get_filename_component(output_directory ${config_header} DIRECTORY)
  add_custom_command(
    OUTPUT ${config_header}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tools/codegen/generate_scenario_config.py
            ${scenario}
            ${config_header}
    DEPENDS
      ${scenario}
      ${CMAKE_SOURCE_DIR}/tools/codegen/generate_scenario_config.py
      ${CMAKE_SOURCE_DIR}/tools/python/sixsim_gnc/attitude.py
      ${CMAKE_SOURCE_DIR}/tools/python/sixsim_gnc/__init__.py
    VERBATIM
  )
endfunction()

set(SIXSIM_SMOKE_SCENARIO_CONFIG
    ${CMAKE_BINARY_DIR}/generated/smoke/scenario_config.hpp)
set(SIXSIM_MATLAB_COMPARE_SCENARIO_CONFIG
    ${CMAKE_BINARY_DIR}/generated/matlab_compare/scenario_config.hpp)
set(SIXSIM_HAL_ALTIMETER_SCENARIO_CONFIG
    ${CMAKE_BINARY_DIR}/generated/hal_altimeter/scenario_config.hpp)

add_generated_scenario(
  ${SIXSIM_SMOKE_SCENARIO_CONFIG}
  ${CMAKE_SOURCE_DIR}/configs/scenarios/test/smoke.yaml
)

add_generated_scenario(
  ${SIXSIM_MATLAB_COMPARE_SCENARIO_CONFIG}
  ${CMAKE_SOURCE_DIR}/configs/scenarios/test/matlab_compare.yaml
)

add_generated_scenario(
  ${SIXSIM_HAL_ALTIMETER_SCENARIO_CONFIG}
  ${CMAKE_SOURCE_DIR}/configs/scenarios/test/hal_altimeter.yaml
)

add_executable(rigid_body_smoke
  apps/smoke/rigid_body_smoke.cpp
  ${SIXSIM_SMOKE_SCENARIO_CONFIG}
  sim/src/logging.cpp
  sim/src/run_artifacts.cpp
  sim/src/run_simulation.cpp
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
  hal/include
  sim/include
  ${CMAKE_BINARY_DIR}/generated/smoke
)

add_executable(matlab_compare
  apps/smoke/rigid_body_smoke.cpp
  ${SIXSIM_MATLAB_COMPARE_SCENARIO_CONFIG}
  sim/src/logging.cpp
  sim/src/run_artifacts.cpp
  sim/src/run_simulation.cpp
  sim/models/aerodynamics/simple_aerodynamics_model_v1.cpp
  sim/models/aerodynamics/zero_aerodynamics.cpp
  sim/models/atmosphere/constant_atmosphere.cpp
  sim/models/gravity/constant_gravity.cpp
  sim/models/propulsion/hardcoded_thrust_curve_propulsion.cpp
  sim/models/propulsion/zero_propulsion.cpp
  sim/models/wind/constant_wind.cpp
)

target_compile_features(matlab_compare PRIVATE cxx_std_20)

target_include_directories(matlab_compare PRIVATE
  .
  include
  hal/include
  sim/include
  ${CMAKE_BINARY_DIR}/generated/matlab_compare
)

add_executable(hal_altimeter
  apps/smoke/rigid_body_smoke.cpp
  ${SIXSIM_HAL_ALTIMETER_SCENARIO_CONFIG}
  sim/src/logging.cpp
  sim/src/run_artifacts.cpp
  sim/src/run_simulation.cpp
  sim/models/aerodynamics/simple_aerodynamics_model_v1.cpp
  sim/models/atmosphere/constant_atmosphere.cpp
  sim/models/gravity/constant_gravity.cpp
  sim/models/propulsion/hardcoded_thrust_curve_propulsion.cpp
  sim/models/propulsion/zero_propulsion.cpp
  sim/models/wind/constant_wind.cpp
)

target_compile_features(hal_altimeter PRIVATE cxx_std_20)

target_include_directories(hal_altimeter PRIVATE
  .
  include
  hal/include
  sim/include
  ${CMAKE_BINARY_DIR}/generated/hal_altimeter
)
