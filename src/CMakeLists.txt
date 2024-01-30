find_package(Boost REQUIRED COMPONENTS program_options)

if(MC_RTC_MAGNUM_BUILD_INTERFACE)

  set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

  set(implot_DIR ${PROJECT_SOURCE_DIR}/ext/implot)
  set(implot_SRC ${implot_DIR}/implot.h ${implot_DIR}/implot.cpp
                 ${implot_DIR}/implot_items.cpp ${implot_DIR}/implot_demo.cpp)

  set(ImGuizmo_DIR ${PROJECT_SOURCE_DIR}/ext/ImGuizmo)
  set(ImGuizmo_SRC ${ImGuizmo_DIR}/ImGuizmo.h ${ImGuizmo_DIR}/ImGuizmo.cpp)

  add_subdirectory(mc_rtc-imgui)

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
      ${mc_rtc-imgui-SRC}
      ${mc_rtc-imgui-HDR})

  add_executable(${PROJECT_NAME} ${project_SRC} ${ImGuizmo_SRC} ${implot_SRC})

  target_compile_definitions(
    ${PROJECT_NAME}
    PRIVATE -DMAGNUM_IMGUIINTEGRATION_USER_CONFIG="custom_imconfig.h")
  target_include_directories(
    ${PROJECT_NAME} PRIVATE SYSTEM ${CMAKE_CURRENT_LIST_DIR}/imconfig
                            ${ImGuizmo_DIR} ${implot_DIR})
  target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE mc_rtc::mc_control
            mc_rtc::mc_control_client
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
            Magnum::ObjImporter
            Boost::program_options
            Boost::disable_autolinking)

  if(TARGET mc_rtc::mc_rtc_ros)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MC_RTC_HAS_ROS_SUPPORT)
    target_link_libraries(${PROJECT_NAME} PRIVATE mc_rtc::mc_rtc_ros)
  endif()

  # Make the executable a default target to build & run in Visual Studio
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT
                                                        ${PROJECT_NAME})

  install(TARGETS ${PROJECT_NAME} DESTINATION bin)

endif() # if(MC_RTC_MAGNUM_BUILD_INTERFACE)
