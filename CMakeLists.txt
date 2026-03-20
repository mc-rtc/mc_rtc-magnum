cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)

project(
  mc-rtc-magnum
  LANGUAGES CXX
  VERSION 1.0.0)

# XXX: should be set to a directory with all plugins symlinked together for nix
set(MAGNUM_WITH_PLUGINS_LIBDIR
    ""
    CACHE PATH "Path to the directory containing Magnum plugins")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ext/magnum/modules/"
                      ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ext/magnum-integration/modules/"
                      ${CMAKE_MODULE_PATH})

option(MC_RTC_MAGNUM_BUILD_INTERFACE "Build the Magnum interface" OFF)

if(MC_RTC_MAGNUM_BUILD_INTERFACE)
  # Magnum options
  set(CORRADE_BUILD_STATIC
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_BUILD_STATIC
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_BUILD_PLUGINS_STATIC
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_WITH_GLFWAPPLICATION
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_WITH_ANYIMAGEIMPORTER
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_WITH_ANYSCENEIMPORTER
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_WITH_OBJIMPORTER
      ON
      CACHE BOOL "" FORCE)

  add_subdirectory(ext/glfw EXCLUDE_FROM_ALL)
  add_subdirectory(ext/corrade EXCLUDE_FROM_ALL)

  add_subdirectory(ext/magnum EXCLUDE_FROM_ALL)

  set(MAGNUM_WITH_ASSIMPIMPORTER
      ON
      CACHE BOOL "" FORCE)
  set(MAGNUM_WITH_STBIMAGEIMPORTER
      ON
      CACHE BOOL "" FORCE)
  add_subdirectory(ext/magnum-plugins EXCLUDE_FROM_ALL)

  set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ext/imgui)
  set(MAGNUM_WITH_IMGUI
      ON
      CACHE BOOL "" FORCE)
  add_subdirectory(ext/magnum-integration EXCLUDE_FROM_ALL)

  find_package(Magnum REQUIRED GL)
  find_package(MagnumIntegration REQUIRED ImGui)

  if(CORRADE_TARGET_ANDROID)
    find_package(Magnum REQUIRED AndroidApplication)
  elseif(CORRADE_TARGET_EMSCRIPTEN)
    find_package(Magnum REQUIRED EmscriptenApplication)
  else()
    find_package(Magnum REQUIRED GlfwApplication)
  endif()
endif()

# mc_rtc find_package(mc_rtc REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Boost REQUIRED COMPONENTS program_options) # todo use CLI11

find_package(mc_rtc-imgui REQUIRED)
# XXX: the official ImGuizmo does not provide cmake export we rely on a fork
# with jrl-cmakemodules export
find_package(ImGuizmo REQUIRED)

find_package(
  Magnum REQUIRED
  COMPONENTS GlfwApplication
             GL
             Magnum
             Primitives
             Shaders
             MeshTools
             SceneGraph
             AnyImageImporter
             AnySceneImporter)
# TODO pacakge in nix add_subdirectory(ext/magnum-integration EXCLUDE_FROM_ALL)
find_package(MagnumIntegration REQUIRED COMPONENTS ImGui)
find_package(MagnumPlugins REQUIRED COMPONENTS AssimpImporter StbImageImporter)

add_subdirectory(src)
