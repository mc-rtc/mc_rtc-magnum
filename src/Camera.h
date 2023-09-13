#pragma once

#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/VertexColorGL.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>

#ifdef CORRADE_TARGET_ANDROID
#  include <Magnum/Platform/AndroidApplication.h>
#elif defined(CORRADE_TARGET_EMSCRIPTEN)
#  include <Magnum/Platform/EmscriptenApplication.h>
#else
#  include <Magnum/Platform/GlfwApplication.h>
#endif

namespace mc_rtc::magnum
{

using namespace Magnum;
using namespace Math::Literals;

using Transform3D = SceneGraph::MatrixTransformation3D;
using Object3D = SceneGraph::Object<Transform3D>;
using Scene3D = SceneGraph::Scene<Transform3D>;

struct Camera
{
  using KeyEvent = Platform::Application::KeyEvent;
  using MouseEvent = Platform::Application::MouseEvent;
  using MouseMoveEvent = Platform::Application::MouseMoveEvent;
  using MouseScrollEvent = Platform::Application::MouseScrollEvent;
  using ViewportEvent = Platform::Application::ViewportEvent;

  Camera(Platform::Application & app, Scene3D & scene);

  inline SceneGraph::Camera3D * camera() const noexcept { return camera_; }

  inline bool isOrthographic() const noexcept { return orthographic_; }

  bool keyPressEvent(Platform::Application & app, KeyEvent & event);
  bool keyReleaseEvent(Platform::Application & app, KeyEvent & event);
  bool mousePressEvent(Platform::Application & app, MouseEvent & event);
  bool mouseMoveEvent(Platform::Application & app, MouseMoveEvent & event);
  bool mouseScrollEvent(Platform::Application & app, MouseScrollEvent & event);
  void viewportEvent(ViewportEvent & event);

private:
  Object3D * object_;
  SceneGraph::Camera3D * camera_;

  bool orthographic_ = false;
  Vector2i lastPosition_{-1};
  Vector3 cameraPosition_;
  Vector3 focusPoint_;

  void resetTransform(Platform::Application & app);
  void setTransform(Platform::Application & app);
  void setProjection(const Vector2i & windowSize);
  void swapProjectionType(Platform::Application & app);
};

} // namespace mc_rtc::magnum
