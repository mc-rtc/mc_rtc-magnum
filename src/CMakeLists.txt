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
    widgets/details/TransformBase.h)

add_executable(${PROJECT_NAME} ${project_SRC})

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE -DMAGNUM_IMGUIINTEGRATION_USER_CONFIG="custom_imconfig.h")
target_include_directories(
  ${PROJECT_NAME} PRIVATE SYSTEM ${CMAKE_CURRENT_LIST_DIR}/imconfig
                          ${ImGuizmo_DIR} ${implot_DIR})

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE mc_rtc-imgui::mc_rtc-imgui
          ImGuizmo::ImGuizmo
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
          Boost::program_options
          Boost::disable_autolinking)

if(TARGET mc_rtc::mc_rtc_ros)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MC_RTC_HAS_ROS_SUPPORT)
  target_link_libraries(${PROJECT_NAME} PRIVATE mc_rtc::mc_rtc_ros)

  # Magnum can't load plugins from multiple paths, and in nix we cannot install
  # plugins in the default folder expected by magnum-plugins/corrade. Thus we
  # set at runtime a path to a folder where all plugins are symlinked together
  if(MAGNUM_WITH_PLUGINS_LIBDIR)
    message(
      STATUS
        "Using plugin path MAGNUM_WITH_PLUGINS_LIBDIR=${MAGNUM_WITH_PLUGINS_LIBDIR}"
    )
    target_compile_definitions(
      ${PROJECT_NAME}
      PRIVATE MAGNUM_WITH_PLUGINS_LIBDIR="${MAGNUM_WITH_PLUGINS_LIBDIR}")
  endif()
endif()

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT
                                                      ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
