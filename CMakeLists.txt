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

include(cmake/SixSimScenario.cmake)

file(GLOB_RECURSE SIXSIM_SCENARIOS
  CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/configs/scenarios/*.yaml"
)

list(SORT SIXSIM_SCENARIOS)

foreach(scenario IN LISTS SIXSIM_SCENARIOS)
  sixsim_add_scenario("${scenario}")
endforeach()
