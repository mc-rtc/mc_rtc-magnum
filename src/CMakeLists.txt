# set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

# set(implot_DIR ${PROJECT_SOURCE_DIR}/ext/implot) set(implot_SRC
# #${implot_DIR}/implot.h ${implot_DIR}/implot.cpp # implot::implot target
# ${implot_DIR}/implot_items.cpp ${implot_DIR}/implot_demo.cpp)

# Not in nix
set(ImGuizmo_DIR ${PROJECT_SOURCE_DIR}/ext/ImGuizmo)
set(ImGuizmo_SRC ${ImGuizmo_DIR}/ImGuizmo.h ${ImGuizmo_DIR}/ImGuizmo.cpp)

# add_subdirectory(mc_rtc-imgui)

set(project_SRC
    McRtcGui.h
    McRtcGui.cpp
    Camera.h
    Camera.cpp
    MagnumClient.h
    MagnumClient.cpp
    Mesh.h
    Mesh.cpp
    Primitives.h
    Primitives.cpp
    widgets/Arrow.h
    widgets/Force.h
    widgets/Point3D.cpp
    widgets/Point3D.h
    widgets/Polygon.h
    widgets/Polygon.cpp
    widgets/Polyhedron.cpp
    widgets/Polyhedron.h
    widgets/Robot.cpp
    widgets/Rotation.h
    widgets/Trajectory.h
    widgets/Transform.h
    widgets/utils.h
    widgets/Visual.h
    widgets/Visual.cpp
    widgets/Widget.h
    widgets/XYTheta.h
    widgets/details/InteractiveMarker.cpp
    widgets/details/InteractiveMarker.h
    widgets/details/TransformBase.h
    # already in mc_rtc-imgui target ${mc_rtc-imgui-SRC} ${mc_rtc-imgui-HDR}
)

add_executable(${PROJECT_NAME} ${project_SRC} ${ImGuizmo_SRC})

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE -DMAGNUM_IMGUIINTEGRATION_USER_CONFIG="custom_imconfig.h")
target_include_directories(
  ${PROJECT_NAME} PRIVATE SYSTEM ${CMAKE_CURRENT_LIST_DIR}/imconfig
                          ${ImGuizmo_DIR} ${implot_DIR})

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE mc_rtc-imgui::mc_rtc-imgui
          # provided by Findmagnum.cmake
          Magnum::Application
          Magnum::GL
          Magnum::Magnum
          Magnum::Primitives
          Magnum::Shaders
          Magnum::MeshTools
          Magnum::SceneGraph
          MagnumIntegration::ImGui
          Magnum::AnyImageImporter
          Magnum::AnySceneImporter
          MagnumPlugins::AssimpImporter
          MagnumPlugins::StbImageImporter
          # Magnum::ImGuiIntegration
          Boost::program_options
          Boost::disable_autolinking)

# target_link_libraries( ${PROJECT_NAME} PRIVATE mc_rtc::mc_control
# mc_rtc::mc_control_client Magnum::ObjImporter Boost::program_options
# Boost::disable_autolinking)

if(TARGET mc_rtc::mc_rtc_ros)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MC_RTC_HAS_ROS_SUPPORT)
  target_link_libraries(${PROJECT_NAME} PRIVATE mc_rtc::mc_rtc_ros)

  # XXX this gets the path to an individual plugin directory in nix store,
  # however magnum does not have a way to load plugins from multiple paths. Thus
  # we use the symlinked version below message(STATUS
  # "MAGNUMPLUGINS_ASSIMPIMPORTER_LIBRARY_RELEASE=${MAGNUMPLUGINS_ASSIMPIMPORTER_LIBRARY_RELEASE}")
  # # Extract the directory get_filename_component(MAGNUM_ASSIMP_IMPORTER_DIR
  # "${MAGNUMPLUGINS_ASSIMPIMPORTER_LIBRARY_RELEASE}" DIRECTORY) #
  # set(MAGNUM_ASSIMP_IMPORTER_DIR
  # "/nix/store/7ra3f9prqc3m4zrhmr6wmbc4gpgb8gcz-magnum-with-plugins/lib/magnum/importers")
  # message(STATUS "MAGNUM_ASSIMP_IMPORTER_DIR=${MAGNUM_ASSIMP_IMPORTER_DIR}") #
  # Pass it to your C++ code as a definition
  # target_compile_definitions(${PROJECT_NAME} PRIVATE
  # MAGNUM_ASSIMP_IMPORTER_DIR="${MAGNUM_ASSIMP_IMPORTER_DIR}" )

  # XXX: This must be passed externally to a plugin path containing a symlink to
  # all plugins
  if(MAGNUM_WITH_PLUGINS_LIBDIR)
    message(
      WARNING "Using MAGNUM_WITH_PLUGINS_LIBDIR=${MAGNUM_WITH_PLUGINS_LIBDIR}")
    target_compile_definitions(
      ${PROJECT_NAME}
      PRIVATE MAGNUM_WITH_PLUGINS_LIBDIR="${MAGNUM_WITH_PLUGINS_LIBDIR}")
  endif()
endif()

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT
                                                      ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
