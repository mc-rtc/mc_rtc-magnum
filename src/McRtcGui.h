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
#include <Magnum/Shaders/VertexColor.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
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

#include "Camera.h"
#include "Client.h"
#include "ImGuizmo.h"

using namespace Magnum;
using namespace Math::Literals;

using Transform3D = SceneGraph::MatrixTransformation3D;
using Object3D = SceneGraph::Object<Transform3D>;
using Scene3D = SceneGraph::Scene<Transform3D>;

struct McRtcGui : public Platform::Application
{
  explicit McRtcGui(const Arguments & arguments);

  void drawEvent() override;

  void viewportEvent(ViewportEvent & event) override;

  void keyPressEvent(KeyEvent & event) override;
  void keyReleaseEvent(KeyEvent & event) override;

  void mousePressEvent(MouseEvent & event) override;
  void mouseReleaseEvent(MouseEvent & event) override;
  void mouseMoveEvent(MouseMoveEvent & event) override;
  void mouseScrollEvent(MouseScrollEvent & event) override;
  void textInputEvent(TextInputEvent & event) override;

  std::unique_ptr<Object3D> loadMesh(const std::string & path);

  void drawCube(Vector3 center, Matrix3 ori, Vector3 size, Color4 color);

  void drawSphere(Vector3 center, float radius, Color4 color);

  void drawFrame(Matrix4 pos, float scale = 0.15);

  void drawLine(Vector3 start, Vector3 end, Color4 color, float thickness = 1.0);

  void drawArrow(Vector3 start, Vector3 end, float shaft_diam, float head_diam, float head_len, Color4 color);

  inline const Camera & camera() const noexcept
  {
    return *camera_;
  }

private:
  ImGuiIntegration::Context imgui_{NoCreate};

  Scene3D scene_;
  SceneGraph::DrawableGroup3D drawables_;
  Containers::Optional<Camera> camera_;

  std::unique_ptr<Object3D> root_;
  std::unique_ptr<Object3D> root2_;

  PluginManager::Manager<Trade::AbstractImporter> manager_;
  Containers::Pointer<Trade::AbstractImporter> importer_;
  Shaders::Phong colorShader_;
  Shaders::Phong textureShader_{Shaders::Phong::Flag::DiffuseTexture};

  struct ImportedMesh
  {
    Containers::Array<Containers::Optional<GL::Mesh>> meshes_;
    Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials_;
    Containers::Array<Containers::Optional<GL::Texture2D>> textures_;
    Containers::Optional<Trade::SceneData> scene_;
    Containers::Array<Containers::Pointer<Trade::ObjectData3D>> objects_;
  };
  std::unordered_map<std::string, ImportedMesh> importedData_;

  ImportedMesh & importData(const std::string & mesh);

  void addObject(ImportedMesh & data, Object3D & parent, UnsignedInt i);

  GL::Mesh cubeMesh_;
  GL::Mesh sphereMesh_;
  GL::Mesh axisMesh_;
  Shaders::Phong shader_;
  Shaders::VertexColor3D vertexShader_;

  Client client_;

  void draw(GL::Mesh & mesh, const Color4 & color, const Matrix4 & worldTransform = {});
};
