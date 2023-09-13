#pragma once

#include "imgui.h"

#include "ImGuizmo.h"
#include "MagnumClient.h"

#include "Camera.h"
#include "Mesh.h"

namespace mc_rtc::magnum
{

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

  std::shared_ptr<Mesh> loadMesh(const std::string & path,
                                 Color4 color,
                                 Object3D * parent = nullptr,
                                 SceneGraph::DrawableGroup3D * group = nullptr);

  BoxPtr makeBox(Vector3 center,
                 Matrix3 ori,
                 Vector3 size,
                 Color4 color,
                 Object3D * parent = nullptr,
                 SceneGraph::DrawableGroup3D * group = nullptr);

  SpherePtr makeSphere(Vector3 center,
                       float radius,
                       Color4 color,
                       Object3D * parent = nullptr,
                       SceneGraph::DrawableGroup3D * group = nullptr);

  EllipsoidPtr makeEllipsoid(Vector3 center,
                             Matrix3 ori,
                             Vector3 size,
                             Color4 color,
                             Object3D * parent = nullptr,
                             SceneGraph::DrawableGroup3D * group = nullptr);

  void drawFrame(Matrix4 pos, float scale = 0.15);

  void drawLine(Vector3 start, Vector3 end, Color4 color, float thickness = 1.0);

  void drawArrow(Vector3 start, Vector3 end, float shaft_diam, float head_diam, float head_len, Color4 color);

  inline const Camera & camera() const noexcept { return *camera_; }

  inline Scene3D & scene() noexcept { return scene_; }

  inline SceneGraph::DrawableGroup3D & drawables() noexcept { return drawables_; }

private:
  ImGuiIntegration::Context imgui_{NoCreate};

  Scene3D scene_;
  SceneGraph::DrawableGroup3D drawables_;
  Containers::Optional<Camera> camera_;

  PluginManager::Manager<Trade::AbstractImporter> manager_;
  Containers::Pointer<Trade::AbstractImporter> importer_;
  Shaders::PhongGL colorShader_;
  Shaders::PhongGL textureShader_{Shaders::PhongGL::Configuration{}.setFlags(Shaders::PhongGL::Flag::DiffuseTexture)};

  std::unordered_map<std::string, ImportedMesh> importedData_;

  ImportedMesh & importData(const std::string & mesh);

  GL::Mesh cubeMesh_;
  GL::Mesh sphereMesh_;
  GL::Mesh axisMesh_;
  Shaders::PhongGL shader_;
  Shaders::VertexColorGL3D vertexShader_;

  MagnumClient client_;

  void draw(GL::Mesh & mesh, const Color4 & color, const Matrix4 & worldTransform = {});
};

} // namespace mc_rtc::magnum
