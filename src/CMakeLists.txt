set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

set(arcball_DIR ${PROJECT_SOURCE_DIR}/ext/magnum-examples/src/arcball)
set(arcball_SRC ${arcball_DIR}/ArcBall.cpp ${arcball_DIR}/ArcBallCamera.h ${arcball_DIR}/ArcBall.h)

set(ImGuizmo_DIR ${PROJECT_SOURCE_DIR}/ext/ImGuizmo)
set(ImGuizmo_SRC ${ImGuizmo_DIR}/ImGuizmo.h ${ImGuizmo_DIR}/ImGuizmo.cpp)

add_executable(${PROJECT_NAME} main.cpp ${arcball_SRC} ${ImGuizmo_SRC})
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM ${arcball_DIR} ${ImGuizmo_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE
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

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
