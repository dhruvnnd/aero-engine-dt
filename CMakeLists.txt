cmake_minimum_required(VERSION 3.20)
project(aero_engine_dt C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# ---------------------------------------------------------------------------
# SDL3 Configuration
# ---------------------------------------------------------------------------
if(EMSCRIPTEN)
  # Emscripten uses its native port system. We create an interface target 
  # matching your app's structure to pass the correct compiler/linker flags.
  add_library(SDL3::SDL3 INTERFACE IMPORTED)
  target_compile_options(SDL3::SDL3 INTERFACE "-sUSE_SDL=3")
  target_link_options(SDL3::SDL3 INTERFACE "-sUSE_SDL=3")
else()
  # Desktop configuration (unchanged)
  set(SDL3_VENDOR_DIR "${CMAKE_SOURCE_DIR}/x86_64-w64-mingw32")
  find_package(SDL3 QUIET CONFIG)
  message(STATUS "SDL3_FOUND=${SDL3_FOUND}  SDL3_DIR=${SDL3_DIR}")

  if(NOT SDL3_FOUND)
    if(EXISTS "${SDL3_VENDOR_DIR}/include/SDL3/SDL.h")
      message(STATUS "find_package didn't resolve SDL3 -- using vendored kit at ${SDL3_VENDOR_DIR} directly")
      add_library(SDL3::SDL3 SHARED IMPORTED)
      set_target_properties(SDL3::SDL3 PROPERTIES
        IMPORTED_IMPLIB             "${SDL3_VENDOR_DIR}/lib/libSDL3.dll.a"
        INTERFACE_INCLUDE_DIRECTORIES "${SDL3_VENDOR_DIR}/include"
      )
    else()
      message(FATAL_ERROR
        "SDL3 not found. Either install an SDL3 dev package (Linux) or "
        "download SDL3-devel-<version>-mingw.zip from "
        "https://libsdl.org/release/ and extract it to "
        "${SDL3_VENDOR_DIR} (see README.md).")
    endif()
  endif()

  if(WIN32 AND EXISTS "${SDL3_VENDOR_DIR}/bin/SDL3.dll")
    set(SDL3_DLL "${SDL3_VENDOR_DIR}/bin/SDL3.dll")
  endif()
endif()

# ---------------------------------------------------------------------------
# sqlite3 Configuration
# ---------------------------------------------------------------------------
set(SQLITE3_VENDOR_DIR "${CMAKE_SOURCE_DIR}/third_party/sqlite3/sqlite-amalgamation-3530400")

if(EXISTS "${SQLITE3_VENDOR_DIR}/sqlite3.c")
  add_library(sqlite3 STATIC "${SQLITE3_VENDOR_DIR}/sqlite3.c")
  target_include_directories(sqlite3 PUBLIC "${SQLITE3_VENDOR_DIR}")
  target_compile_options(sqlite3 PRIVATE -O2)
  
  if(WIN32)
    target_compile_definitions(sqlite3 PRIVATE _CRT_SECURE_NO_WARNINGS)
  endif()
  
  if(EMSCRIPTEN)
    # Required if your project performs filesystem operations via SQLite.
    # SQLITE_OMIT_LOAD_EXTENSION prevents dynamic library loading errors on the web.
    target_compile_definitions(sqlite3 PRIVATE SQLITE_OMIT_LOAD_EXTENSION)
  endif()
else()
  message(FATAL_ERROR
    "sqlite3 amalgamation not found at ${SQLITE3_VENDOR_DIR}. Download "
    "sqlite-amalgamation-<version>.zip from https://sqlite.org/download.html "
    "and extract it under third_party/sqlite3/ (see tools/README).")
endif()

# ---------------------------------------------------------------------------
# engine_core
# ---------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/math/*.c"
  "${CMAKE_SOURCE_DIR}/src/physics/*.c"
  "${CMAKE_SOURCE_DIR}/src/telemetry/*.c"
  "${CMAKE_SOURCE_DIR}/src/model/*.c"
  "${CMAKE_SOURCE_DIR}/src/health/*.c"
  "${CMAKE_SOURCE_DIR}/src/ml/*.c"
  "${CMAKE_SOURCE_DIR}/src/mission/*.c"
  "${CMAKE_SOURCE_DIR}/src/data/*.c"
  "${CMAKE_SOURCE_DIR}/src/util/*.c"
)

if(CORE_SOURCES)
  add_library(engine_core STATIC ${CORE_SOURCES})
  target_include_directories(engine_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
  target_compile_options(engine_core PRIVATE -Wall -Wextra)
  target_link_libraries(engine_core PUBLIC sqlite3)
else()
  add_library(engine_core INTERFACE)
  target_include_directories(engine_core INTERFACE ${CMAKE_SOURCE_DIR}/src)
  target_link_libraries(engine_core INTERFACE sqlite3)
endif()

# ---------------------------------------------------------------------------
# aero_engine_dt (Main Application)
# ---------------------------------------------------------------------------
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/platform/*.c"
  "${CMAKE_SOURCE_DIR}/src/ui/*.c"
)
list(APPEND APP_SOURCES "${CMAKE_SOURCE_DIR}/src/main.c")

add_executable(aero_engine_dt ${APP_SOURCES})

target_include_directories(aero_engine_dt PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(aero_engine_dt PRIVATE engine_core SDL3::SDL3)

target_compile_options(aero_engine_dt PRIVATE
  -Wall -Wextra
  $<$<CONFIG:Debug>:-g -O0>
  $<$<CONFIG:Release>:-O2>
)

if(EMSCRIPTEN)
  # Forces Emscripten to output index.html, index.js, and index.wasm
  set_target_properties(aero_engine_dt PROPERTIES SUFFIX ".html")
  
  # Crucial WebAssembly settings:
  # 1. ALLOW_MEMORY_GROWTH lets SQLite and your application allocate space as needed.
  # 2. USE_WEBGL2=1 ensures SDL3 rendering works with standard web graphics pipelines.
  target_link_options(aero_engine_dt PRIVATE 
    "-sALLOW_MEMORY_GROWTH=1"
    "-sUSE_WebGL2=1"
  )
else()
  # Desktop link settings (unchanged)
  if(WIN32)
    target_link_options(aero_engine_dt PRIVATE $<$<CONFIG:Release>:-mwindows>)
  endif()

  if(WIN32 AND DEFINED SDL3_DLL)
    add_custom_command(TARGET aero_engine_dt POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${SDL3_DLL}" "$<TARGET_FILE_DIR:aero_engine_dt>"
      COMMENT "Copying SDL3.dll next to the executable"
    )
  endif()
endif()

# ---------------------------------------------------------------------------
# CLI Tools & Tests (Desktop Only)
# ---------------------------------------------------------------------------
if(NOT EMSCRIPTEN)
  if(EXISTS "${CMAKE_SOURCE_DIR}/tools/twin_sim.c")
    add_executable(twin_sim "${CMAKE_SOURCE_DIR}/tools/twin_sim.c")
    target_link_libraries(twin_sim PRIVATE engine_core)
    target_compile_options(twin_sim PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g -O0> $<$<CONFIG:Release>:-O2>)
  endif()

  if(EXISTS "${CMAKE_SOURCE_DIR}/tools/twin_config.c")
    add_executable(twin_config "${CMAKE_SOURCE_DIR}/tools/twin_config.c")
    target_link_libraries(twin_config PRIVATE engine_core)
    target_compile_options(twin_config PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g -O0> $<$<CONFIG:Release>:-O2>)
  endif()

  if(EXISTS "${CMAKE_SOURCE_DIR}/tests/CMakeLists.txt")
    enable_testing()
    add_subdirectory(tests)
  endif()
endif()
