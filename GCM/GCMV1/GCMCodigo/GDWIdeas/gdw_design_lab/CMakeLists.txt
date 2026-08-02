cmake_minimum_required(VERSION 3.16)
project(GDWDesignLab LANGUAGES CXX)

# ── Standard / build type ──────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(Threads REQUIRED)

# ═══════════════════════════════════════════════════════════════════════════
#  Headless optimizer  (this target is the verified core: no external deps)
# ═══════════════════════════════════════════════════════════════════════════
add_executable(gdw_design_lab src/main_headless.cpp)
target_include_directories(gdw_design_lab PRIVATE src)
target_link_libraries(gdw_design_lab PRIVATE Threads::Threads)
if(MSVC)
  target_compile_options(gdw_design_lab PRIVATE /O2)
else()
  target_compile_options(gdw_design_lab PRIVATE -O2)
endif()

# ═══════════════════════════════════════════════════════════════════════════
#  GUI  (SDL2 + Dear ImGui)  —  OPTIONAL, OFF by default
#
#  Enable with:  cmake -DBUILD_GUI=ON ..
#
#  NOTE: the GUI sources were written but could NOT be compiled in the
#  author's sandbox (no SDL2 / ImGui / network there). Expect minor local
#  fix-ups. SDL2 must be installed; Dear ImGui is fetched automatically.
# ═══════════════════════════════════════════════════════════════════════════
option(BUILD_GUI "Build the SDL2 + Dear ImGui visualizer" OFF)

if(BUILD_GUI)
  find_package(SDL2 REQUIRED)

  include(FetchContent)
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.90.9
  )
  FetchContent_MakeAvailable(imgui)

  add_library(imgui_sdl STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
  )
  target_include_directories(imgui_sdl PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )
  target_link_libraries(imgui_sdl PUBLIC SDL2::SDL2)

  add_executable(gdw_design_lab_gui src/gui_main.cpp)
  target_include_directories(gdw_design_lab_gui PRIVATE src)
  target_link_libraries(gdw_design_lab_gui PRIVATE imgui_sdl SDL2::SDL2 Threads::Threads)
endif()