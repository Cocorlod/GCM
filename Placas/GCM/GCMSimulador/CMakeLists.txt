cmake_minimum_required(VERSION 3.20)
project(GDWDesignLab VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Build options ─────────────────────────────────────────────────────────
option(GDW_HEADLESS "Build without SDL2/ImGui — optimization + CLI only" OFF)
option(GDW_NATIVE   "Enable -march=native (faster on local machine)"      ON)
option(GDW_ASAN     "Enable AddressSanitizer (debug only)"                OFF)

message(STATUS "=================================================")
message(STATUS " GDW Design Lab v1.0  –  Micromouse Optimizer   ")
message(STATUS "=================================================")
message(STATUS " Headless  : ${GDW_HEADLESS}")
message(STATUS " Native    : ${GDW_NATIVE}")

# ── Threads ───────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# ── ImGui + SDL2 ─────────────────────────────────────────────────────────
if(NOT GDW_HEADLESS)
  find_package(SDL2 REQUIRED)

  set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
  if(NOT EXISTS "${IMGUI_DIR}/imgui.h")
    message(FATAL_ERROR
      "\n  ImGui not found at ${IMGUI_DIR}\n"
      "  Run:  git clone --depth 1 --branch v1.90.1 "
      "https://github.com/ocornut/imgui external/imgui\n"
      "  Or build headless:  cmake -DGDW_HEADLESS=ON ..")
  endif()

  set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer2.cpp
  )
  message(STATUS " SDL2       : ${SDL2_VERSION}")
  message(STATUS " ImGui      : found at ${IMGUI_DIR}")
endif()

# ── Core sources (always compiled) ───────────────────────────────────────
set(CORE_SOURCES
  src/core/RobotDesign.cpp
  src/core/MazeEngine.cpp
  src/core/PhysicsEngine.cpp
  src/core/GDWAlgorithm.cpp
  src/core/TrajectoryGen.cpp
  src/simulation/DigitalTwin.cpp
  src/optimization/GeneticAlgorithm.cpp
  src/optimization/FitnessEvaluator.cpp
  src/export/DesignExporter.cpp
)

# ── GUI sources ───────────────────────────────────────────────────────────
if(NOT GDW_HEADLESS)
  set(GUI_SOURCES
    src/gui/App.cpp
    src/gui/MazeRenderer.cpp
    src/gui/OptimizationPanel.cpp
    src/gui/AnalyticsPanel.cpp
    ${IMGUI_SOURCES}
  )
else()
  set(GUI_SOURCES "")
endif()

# ── Executable ────────────────────────────────────────────────────────────
add_executable(GDWDesignLab
  src/main.cpp
  ${CORE_SOURCES}
  ${GUI_SOURCES}
)

target_include_directories(GDWDesignLab PRIVATE src/)

if(NOT GDW_HEADLESS)
  target_include_directories(GDWDesignLab PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${SDL2_INCLUDE_DIRS}
  )
  target_link_libraries(GDWDesignLab PRIVATE ${SDL2_LIBRARIES})
  target_compile_definitions(GDWDesignLab PRIVATE GDW_GUI_ENABLED)
endif()

target_link_libraries(GDWDesignLab PRIVATE Threads::Threads)

# ── Compiler flags ────────────────────────────────────────────────────────
if(MSVC)
  target_compile_options(GDWDesignLab PRIVATE
    /O2 /W4 /wd4996 /wd4267 /wd4244 /wd4305)
  target_compile_definitions(GDWDesignLab PRIVATE
    _CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
  target_compile_options(GDWDesignLab PRIVATE
    -O3 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
  if(GDW_NATIVE)
    target_compile_options(GDWDesignLab PRIVATE -march=native)
  endif()
  if(GDW_ASAN)
    target_compile_options(GDWDesignLab PRIVATE -fsanitize=address,undefined)
    target_link_options   (GDWDesignLab PRIVATE -fsanitize=address,undefined)
  endif()
endif()

# ── Install ───────────────────────────────────────────────────────────────
install(TARGETS GDWDesignLab DESTINATION bin)

message(STATUS "=================================================")
