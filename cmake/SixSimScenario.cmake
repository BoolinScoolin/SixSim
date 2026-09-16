function(sixsim_add_model_component component)
  if(TARGET ${component})
    return()
  endif()

  add_library(${component} STATIC ${ARGN})
  target_compile_features(${component} PUBLIC cxx_std_20)
  target_include_directories(${component} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/flight/include
    ${CMAKE_SOURCE_DIR}/hal/include
    ${CMAKE_SOURCE_DIR}/sim/include
  )
endfunction()

function(sixsim_add_scenario scenario)
  get_filename_component(
    scenario_path "${scenario}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}"
  )
  get_filename_component(target "${scenario_path}" NAME_WE)
  file(RELATIVE_PATH scenario_display "${CMAKE_SOURCE_DIR}" "${scenario_path}")

  if(TARGET ${target})
    get_target_property(existing_scenario ${target} SIXSIM_SCENARIO_PATH)
    if(existing_scenario)
      file(RELATIVE_PATH existing_display
        "${CMAKE_SOURCE_DIR}" "${existing_scenario}"
      )
      message(FATAL_ERROR
        "Scenario target '${target}' has duplicate filename stems:\n"
        "  ${existing_display}\n"
        "  ${scenario_display}"
      )
    endif()
    message(FATAL_ERROR
      "Scenario '${scenario_display}' produces target '${target}', which "
      "collides with an existing non-scenario target."
    )
  endif()

  set(generated_directory "${CMAKE_BINARY_DIR}/generated/${target}")
  set(build_manifest "${generated_directory}/build_manifest.cmake")
  set(config_header "${generated_directory}/scenario_config.hpp")

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tools/codegen/generate_scenario_config.py
            --build-manifest
            ${scenario_path}
            ${build_manifest}
    RESULT_VARIABLE manifest_result
    ERROR_VARIABLE manifest_error
  )

  if(NOT manifest_result EQUAL 0)
    message(FATAL_ERROR
      "Failed to generate ${target} build manifest:\n${manifest_error}"
    )
  endif()

  include(${build_manifest})
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${scenario_path}
    ${SIXSIM_SCENARIO_CONFIGURATION_DEPENDENCIES}
    ${CMAKE_SOURCE_DIR}/tools/codegen/generate_scenario_config.py
    ${CMAKE_SOURCE_DIR}/tools/codegen/fcu_profile.py
  )

  add_custom_command(
    OUTPUT ${config_header}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${generated_directory}
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tools/codegen/generate_scenario_config.py
            ${scenario_path}
            ${config_header}
    DEPENDS
      ${scenario_path}
      ${SIXSIM_SCENARIO_CONFIGURATION_DEPENDENCIES}
      ${CMAKE_SOURCE_DIR}/tools/codegen/generate_scenario_config.py
      ${CMAKE_SOURCE_DIR}/tools/codegen/fcu_profile.py
      ${CMAKE_SOURCE_DIR}/tools/python/sixsim_gnc/attitude.py
      ${CMAKE_SOURCE_DIR}/tools/python/sixsim_gnc/__init__.py
    VERBATIM
  )

  add_executable(${target}
    ${CMAKE_SOURCE_DIR}/apps/smoke/rigid_body_smoke.cpp
    ${config_header}
    ${CMAKE_SOURCE_DIR}/sim/src/logging.cpp
    ${CMAKE_SOURCE_DIR}/sim/src/run_artifacts.cpp
    ${CMAKE_SOURCE_DIR}/sim/src/run_simulation.cpp
  )
  set_property(TARGET ${target} PROPERTY SIXSIM_SCENARIO_PATH ${scenario_path})

  target_link_libraries(${target} PRIVATE
    ${SIXSIM_SCENARIO_MODEL_COMPONENTS}
  )
  target_compile_features(${target} PRIVATE cxx_std_20)
  target_include_directories(${target} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/flight/include
    ${CMAKE_SOURCE_DIR}/hal/include
    ${CMAKE_SOURCE_DIR}/sim/include
    ${generated_directory}
  )
endfunction()
