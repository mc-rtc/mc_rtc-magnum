set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

set(arcball_DIR ${PROJECT_SOURCE_DIR}/ext/magnum-examples/src/arcball)
set(arcball_SRC ${arcball_DIR}/ArcBall.cpp ${arcball_DIR}/ArcBallCamera.h ${arcball_DIR}/ArcBall.h)

set(ImGuizmo_DIR ${PROJECT_SOURCE_DIR}/ext/ImGuizmo)
set(ImGuizmo_SRC ${ImGuizmo_DIR}/ImGuizmo.h ${ImGuizmo_DIR}/ImGuizmo.cpp)

set(project_SRC
  McRtcGui.h
  McRtcGui.cpp
  Camera.h
  Category.h
  Category.cpp
  Client.h
  Client.cpp
  Widget.h
  widgets/ArrayInput.h
  widgets/ArrayLabel.h
  widgets/Arrow.h
  widgets/Button.h
  widgets/Checkbox.h
  widgets/ComboInput.h
  widgets/DataComboInput.h
  widgets/Force.h
  widgets/Form.h
  widgets/IntegerInput.h
  widgets/Label.h
  widgets/NumberInput.h
  widgets/NumberSlider.h
  widgets/Point3D.cpp
  widgets/Point3D.h
  widgets/Polygon.h
  widgets/Robot.h
  widgets/Robot.cpp
  widgets/Rotation.h
  widgets/Schema.cpp
  widgets/Schema.h
  widgets/StringInput.h
  widgets/Table.h
  widgets/Trajectory.h
  widgets/Transform.h
  widgets/utils.h
  widgets/Widget.h
  widgets/XYTheta.h
  widgets/details/InteractiveMarker.cpp
  widgets/details/InteractiveMarker.h
  widgets/details/SingleInput.h
  widgets/details/TransformBase.h
  widgets/form/schema.cpp
  widgets/form/schema.h
  widgets/form/widgets.cpp
  widgets/form/widgets.h
)

add_executable(${PROJECT_NAME} ${project_SRC} ${arcball_SRC} ${ImGuizmo_SRC})

target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM ${arcball_DIR} ${ImGuizmo_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE
    mc_rtc::mc_control
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
)

if(TARGET mc_rtc::mc_rtc_ros)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MC_RTC_HAS_ROS_SUPPORT)
  target_link_libraries(${PROJECT_NAME} PRIVATE mc_rtc::mc_rtc_ros)
endif()

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
