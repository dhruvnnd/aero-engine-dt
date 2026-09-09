# Headless tests for engine_core (physics + model + telemetry). No SDL.
# Each test_*.c is a standalone executable registered with CTest:
#
#   cmake --build build
#   ctest --test-dir build --output-on-failure

set(ENGINE_CORE_TESTS
  test_integrator
  test_combustion
  test_engine_model
  test_thermal
  test_model_sync
  test_sensor
  test_channels
)

foreach(test_name IN LISTS ENGINE_CORE_TESTS)
  add_executable(${test_name} ${test_name}.c)
  # engine_core carries the src/ include path as a PUBLIC usage requirement;
  # CMAKE_CURRENT_SOURCE_DIR is for test_util.h.
  target_link_libraries(${test_name} PRIVATE engine_core)
  target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(${test_name} PRIVATE -Wall -Wextra)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
