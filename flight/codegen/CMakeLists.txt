set(algorithm_generator
  "${CMAKE_CURRENT_LIST_DIR}/generate_algorithm_artifacts.py"
)
set(algorithms_directory "${CMAKE_CURRENT_LIST_DIR}/../src/algorithms")
set(profiles_directory "${CMAKE_CURRENT_LIST_DIR}/../src/profiles")
set(generated_root "${CMAKE_CURRENT_LIST_DIR}/../build/generated")

file(GLOB algorithm_yamls CONFIGURE_DEPENDS
  "${algorithms_directory}/*.yaml"
)

set(generated_algorithm_headers)
foreach(algorithm_yaml IN LISTS algorithm_yamls)
  get_filename_component(algorithm_name "${algorithm_yaml}" NAME_WE)
  set(algorithm_source "${algorithms_directory}/${algorithm_name}.cpp")
  if(NOT EXISTS "${algorithm_source}")
    message(FATAL_ERROR
      "Algorithm YAML '${algorithm_yaml}' has no matching source file: "
      "${algorithm_source}"
    )
  endif()

  set(algorithm_header
    "${generated_root}/include/sixsim/flight/algorithms/${algorithm_name}.hpp"
  )
  add_custom_command(
    OUTPUT "${algorithm_header}"
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            "${algorithm_generator}"
            --algorithm-header-only
            "${algorithm_yaml}"
            "${generated_root}"
    DEPENDS
      "${algorithm_yaml}"
      "${algorithm_generator}"
    VERBATIM
  )
  list(APPEND generated_algorithm_headers "${algorithm_header}")
endforeach()

add_custom_target(sixsim_flight_generated_algorithm_headers
  DEPENDS ${generated_algorithm_headers}
)

function(sixsim_add_flight_profile_codegen profile)
  set(profile_target "sixsim_flight_generated_algorithms_${profile}")
  set(profile_algorithms_property
    "SIXSIM_FLIGHT_PROFILE_ALGORITHMS_${profile}"
  )
  if(TARGET ${profile_target})
    get_property(profile_algorithms GLOBAL PROPERTY
      "${profile_algorithms_property}"
    )
    set(SIXSIM_FLIGHT_PROFILE_ALGORITHMS
      "${profile_algorithms}"
      PARENT_SCOPE
    )
    return()
  endif()

  set(algorithm_generator
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_algorithm_artifacts.py"
  )
  set(algorithms_directory "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../src/algorithms")
  set(profiles_directory "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../src/profiles")
  set(generated_root "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../build/generated")
  set(common_values "${profiles_directory}/${profile}/common_values.hpp")
  set(state_manifest "${profiles_directory}/${profile}/state_machine.yaml")
  if(NOT EXISTS "${common_values}")
    message(FATAL_ERROR
      "FCU profile '${profile}' has no CommonValues header: ${common_values}"
    )
  endif()
  if(NOT EXISTS "${state_manifest}")
    message(FATAL_ERROR
      "FCU profile '${profile}' has no state/algorithm manifest: ${state_manifest}"
    )
  endif()

  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${state_manifest}"
  )
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            "${algorithm_generator}"
            --manifest-algorithms
            "${state_manifest}"
            unused_revision
            unused_output_root
    RESULT_VARIABLE manifest_result
    OUTPUT_VARIABLE manifest_algorithms_output
    ERROR_VARIABLE manifest_error
  )
  if(NOT manifest_result EQUAL 0)
    message(FATAL_ERROR
      "Failed to read flight algorithm manifest '${state_manifest}':\n"
      "${manifest_error}"
    )
  endif()
  string(STRIP "${manifest_algorithms_output}" manifest_algorithms_output)
  string(REPLACE "\n" ";" profile_algorithms "${manifest_algorithms_output}")
  set_property(GLOBAL PROPERTY "${profile_algorithms_property}"
    "${profile_algorithms}"
  )
  set(SIXSIM_FLIGHT_PROFILE_ALGORITHMS
    "${profile_algorithms}"
    PARENT_SCOPE
  )

  set(state_enum
    "${generated_root}/include/sixsim/flight/profiles/${profile}/State.hpp"
  )
  add_custom_command(
    OUTPUT "${state_enum}"
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            "${algorithm_generator}"
            --state-enum-only
            "${state_manifest}"
            "${profile}"
            "${generated_root}"
    DEPENDS
      "${state_manifest}"
      "${algorithm_generator}"
    VERBATIM
  )
  set(generated_profile_artifacts "${state_enum}")
  set(generated_profile_mappings)

  foreach(algorithm_name IN LISTS profile_algorithms)
    set(algorithm_yaml "${algorithms_directory}/${algorithm_name}.yaml")
    get_filename_component(algorithm_name "${algorithm_yaml}" NAME_WE)
    set(algorithm_header
      "${generated_root}/include/sixsim/flight/algorithms/${algorithm_name}.hpp"
    )
    set(mapping_header
      "${generated_root}/include/sixsim/flight/profiles/${profile}/${algorithm_name}Bindings.hpp"
    )
    add_custom_command(
      OUTPUT "${mapping_header}"
      COMMAND ${CMAKE_COMMAND} -E env
              "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
              ${Python3_EXECUTABLE}
              "${algorithm_generator}"
              --mapping-only
              "${algorithm_yaml}"
              "${profile}"
              "${generated_root}"
      DEPENDS
        "${algorithm_yaml}"
        "${algorithm_generator}"
        "${common_values}"
        "${algorithm_header}"
      VERBATIM
    )
    list(APPEND generated_profile_artifacts "${mapping_header}")
    list(APPEND generated_profile_mappings "${mapping_header}")
  endforeach()

  set(profile_bindings_header
    "${generated_root}/include/sixsim/flight/profiles/${profile}/AlgorithmBindings.hpp"
  )
  add_custom_command(
    OUTPUT "${profile_bindings_header}"
    COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=${SIXSIM_CODEGEN_PYTHONPATH}"
            ${Python3_EXECUTABLE}
            "${algorithm_generator}"
            --profile-bindings-only
            "${state_manifest}"
            "${profile}"
            "${generated_root}"
    DEPENDS
      "${state_manifest}"
      "${algorithm_generator}"
      ${generated_profile_mappings}
    VERBATIM
  )
  list(APPEND generated_profile_artifacts "${profile_bindings_header}")

  add_custom_target(${profile_target} DEPENDS ${generated_profile_artifacts})
  add_dependencies(${profile_target} sixsim_flight_generated_algorithm_headers)
endfunction()
