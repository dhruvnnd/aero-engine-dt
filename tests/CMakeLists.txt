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
  test_cylinder
  test_fuel
  test_lubrication
  test_electrical
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

# ---------------------------------------------------------------------------
# UI tests -- the SDL-free pieces of src/ui (layout math, history ring). Each
# compiles its one dependency directly rather than linking a library; there is
# no ui_core target and these sources pull in no SDL.
# ---------------------------------------------------------------------------
set(UI_TESTS
  test_ui_layout
  test_ui_history
)
set(test_ui_layout_SRC  ${CMAKE_SOURCE_DIR}/src/ui/ui_layout.c)
set(test_ui_history_SRC ${CMAKE_SOURCE_DIR}/src/ui/ui_history.c)

foreach(test_name IN LISTS UI_TESTS)
  add_executable(${test_name} ${test_name}.c ${${test_name}_SRC})
  target_include_directories(${test_name} PRIVATE
    ${CMAKE_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(${test_name} PRIVATE -Wall -Wextra)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
