#pragma once

#include "Client.h"
#include "ImGuizmo.h"

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

  std::shared_ptr<Mesh> loadMesh(const std::string & path);

  void drawCube(Vector3 center, Matrix3 ori, Vector3 size, Color4 color);

  void drawSphere(Vector3 center, float radius, Color4 color);

  void drawFrame(Matrix4 pos, float scale = 0.15);

  void drawLine(Vector3 start, Vector3 end, Color4 color, float thickness = 1.0);

  void drawArrow(Vector3 start, Vector3 end, float shaft_diam, float head_diam, float head_len, Color4 color);

  inline const Camera & camera() const noexcept
  {
    return *camera_;
  }

  inline Scene3D & scene() noexcept
  {
    return scene_;
  }

private:
  ImGuiIntegration::Context imgui_{NoCreate};

  Scene3D scene_;
  SceneGraph::DrawableGroup3D drawables_;
  Containers::Optional<Camera> camera_;

  PluginManager::Manager<Trade::AbstractImporter> manager_;
  Containers::Pointer<Trade::AbstractImporter> importer_;
  Shaders::Phong colorShader_;
  Shaders::Phong textureShader_{Shaders::Phong::Flag::DiffuseTexture};

  std::unordered_map<std::string, ImportedMesh> importedData_;

  ImportedMesh & importData(const std::string & mesh);

  GL::Mesh cubeMesh_;
  GL::Mesh sphereMesh_;
  GL::Mesh axisMesh_;
  Shaders::Phong shader_;
  Shaders::VertexColor3D vertexShader_;

  Client client_;

  void draw(GL::Mesh & mesh, const Color4 & color, const Matrix4 & worldTransform = {});
};

} // namespace mc_rtc::magnum
