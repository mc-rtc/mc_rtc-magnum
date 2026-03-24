cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)

project(
  mc-rtc-magnum
  LANGUAGES CXX
  VERSION 1.0.0)

# should be set to a directory with all plugins symlinked together for nix
set(MAGNUM_WITH_PLUGINS_LIBDIR
    ""
    CACHE PATH "Path to the directory containing Magnum plugins")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ext/magnum/modules/"
                      ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/ext/magnum-integration/modules/"
                      ${CMAKE_MODULE_PATH})

find_package(Boost REQUIRED COMPONENTS program_options) # todo use CLI11

find_package(mc_rtc-imgui REQUIRED)
# XXX: the official ImGuizmo does not provide cmake export we rely on a fork
# with jrl-cmakemodules export
find_package(ImGuizmo REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
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
find_package(MagnumIntegration REQUIRED COMPONENTS ImGui)
find_package(MagnumPlugins REQUIRED COMPONENTS AssimpImporter StbImageImporter)

add_subdirectory(src)
