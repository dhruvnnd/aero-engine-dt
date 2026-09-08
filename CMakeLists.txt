cmake_minimum_required(VERSION 3.20)
project(aero_engine_dt C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# ---------------------------------------------------------------------------
# SDL3
#
# Preferred: a system/toolchain-provided SDL3 (e.g. a Linux edge/SocketCAN
# build with SDL3 installed via package manager or find_package config).
# Fallback: the vendored SDL3-devel mingw kit, extracted to
# <repo root>/x86_64-w64-mingw32 per README.md (gitignored, not committed).
# ---------------------------------------------------------------------------
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

# Whichever branch above resolved SDL3::SDL3 -- find_package() picking up
# the vendored kit's own SDL3Config.cmake (e.g. via a PATH entry compile.ps1
# added), or our manual IMPORTED-target fallback -- the DLL still needs to
# sit next to the exe at runtime if it's *this repo's* vendored kit. Checking
# for the file directly, rather than which branch ran, means the copy step
# fires either way.
if(WIN32 AND EXISTS "${SDL3_VENDOR_DIR}/bin/SDL3.dll")
  set(SDL3_DLL "${SDL3_VENDOR_DIR}/bin/SDL3.dll")
endif()

# ---------------------------------------------------------------------------
# twin_core: math/physics/telemetry/health/ml/mission/twin/data/util.
# Pure C, no SDL dependency by convention -- this is what tests/ links
# against so the physics model is testable headless.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/math/*.c"
  "${CMAKE_SOURCE_DIR}/src/physics/*.c"
  "${CMAKE_SOURCE_DIR}/src/telemetry/*.c"
  "${CMAKE_SOURCE_DIR}/src/twin/*.c"
  "${CMAKE_SOURCE_DIR}/src/health/*.c"
  "${CMAKE_SOURCE_DIR}/src/ml/*.c"
  "${CMAKE_SOURCE_DIR}/src/mission/*.c"
  "${CMAKE_SOURCE_DIR}/src/data/*.c"
  "${CMAKE_SOURCE_DIR}/src/util/*.c"
)

if(CORE_SOURCES)
  add_library(twin_core STATIC ${CORE_SOURCES})
  target_include_directories(twin_core PUBLIC ${CMAKE_SOURCE_DIR}/src)
  target_compile_options(twin_core PRIVATE -Wall -Wextra)
else()
  # Nothing under those directories yet -- keep the target valid so the
  # rest of the build still links cleanly while the project is scaffolded.
  # (INTERFACE targets can't take PRIVATE compile options, so this branch
  # just skips warning flags until there's actually a .c file to apply them to.)
  add_library(twin_core INTERFACE)
  target_include_directories(twin_core INTERFACE ${CMAKE_SOURCE_DIR}/src)
endif()

# ---------------------------------------------------------------------------
# aero_engine_dt: platform/sdl + ui + main.c. Only place SDL3 is linked.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/platform/*.c"
  "${CMAKE_SOURCE_DIR}/src/ui/*.c"
)
list(APPEND APP_SOURCES "${CMAKE_SOURCE_DIR}/src/main.c")

add_executable(aero_engine_dt ${APP_SOURCES})

target_include_directories(aero_engine_dt PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(aero_engine_dt PRIVATE twin_core SDL3::SDL3)

target_compile_options(aero_engine_dt PRIVATE
  -Wall -Wextra
  $<$<CONFIG:Debug>:-g -O0>
  $<$<CONFIG:Release>:-O2>
)

if(WIN32)
  target_link_options(aero_engine_dt PRIVATE $<$<CONFIG:Release>:-mwindows>)
endif()

# Vendored builds ship a DLL that has to sit next to the exe at runtime.
if(WIN32 AND DEFINED SDL3_DLL)
  add_custom_command(TARGET aero_engine_dt POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SDL3_DLL}" "$<TARGET_FILE_DIR:aero_engine_dt>"
    COMMENT "Copying SDL3.dll next to the executable"
  )
endif()

# ---------------------------------------------------------------------------
# Tests -- only wired up once tests/CMakeLists.txt exists.
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_SOURCE_DIR}/tests/CMakeLists.txt")
  enable_testing()
  add_subdirectory(tests)
endif()
