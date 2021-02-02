#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/ImGuiIntegration/Context.hpp>
#include <Magnum/Math/Color.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Primitives/Axis.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Grid.h>
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

#include "ArcBallCamera.h"
#include "ImGuizmo.h"

using namespace Magnum;
using namespace Math::Literals;

using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;

class ColoredDrawable : public SceneGraph::Drawable3D
{
public:
  explicit ColoredDrawable(Object3D & object,
                           Shaders::Phong & shader,
                           GL::Mesh & mesh,
                           const Color4 & color,
                           SceneGraph::DrawableGroup3D & group)
  : SceneGraph::Drawable3D{object, &group}, shader_(shader), mesh_(mesh), color_{color}
  {
  }

private:
  void draw(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override
  {
    shader_.setDiffuseColor(color_)
        .setLightPositions({{camera.cameraMatrix().transformPoint({10.0f, 10.0f, 10.0f}), 0.0f}})
        .setTransformationMatrix(transformationMatrix)
        .setNormalMatrix(transformationMatrix.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .draw(mesh_);
  }

  Shaders::Phong & shader_;
  GL::Mesh & mesh_;
  Color4 color_;
};

class TexturedDrawable : public SceneGraph::Drawable3D
{
public:
  explicit TexturedDrawable(Object3D & object,
                            Shaders::Phong & shader,
                            GL::Mesh & mesh,
                            GL::Texture2D & texture,
                            SceneGraph::DrawableGroup3D & group)
  : SceneGraph::Drawable3D{object, &group}, shader_(shader), mesh_(mesh), texture_(texture)
  {
  }

private:
  void draw(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override
  {
    shader_.setLightPositions({{camera.cameraMatrix().transformPoint({10.0f, 10.0f, 10.0f}), 0.0f}})
        .setTransformationMatrix(transformationMatrix)
        .setNormalMatrix(transformationMatrix.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .bindDiffuseTexture(texture_)
        .draw(mesh_);
  }

  Shaders::Phong & shader_;
  GL::Mesh & mesh_;
  GL::Texture2D & texture_;
};

class McRtcGui : public Platform::Application
{
public:
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

  bool loadMesh(const std::string & path);

  void addObject(Trade::AbstractImporter & importer,
                 Containers::ArrayView<const Containers::Optional<Trade::PhongMaterialData>> materials,
                 Object3D & parent,
                 UnsignedInt i);

private:
  ImGuiIntegration::Context imgui_{NoCreate};

  Scene3D scene_;
  SceneGraph::DrawableGroup3D drawables_;
  Containers::Optional<Examples::ArcBallCamera> camera_;

  PluginManager::Manager<Trade::AbstractImporter> manager_;
  Containers::Pointer<Trade::AbstractImporter> importer_;
  Shaders::Phong colorShader_;
  Shaders::Phong textureShader_{Shaders::Phong::Flag::DiffuseTexture};
  Containers::Array<Containers::Optional<GL::Mesh>> meshes_;
  Containers::Array<Containers::Optional<GL::Texture2D>> textures_;
  Object3D root_;

  ImGuizmo::OPERATION guizmoOperation_ = ImGuizmo::TRANSLATE;
  ImGuizmo::MODE guizmoMode_ = ImGuizmo::LOCAL;
};

struct Grid : public SceneGraph::Drawable3D
{
  Grid(Object3D & object, SceneGraph::DrawableGroup3D & drawables) : SceneGraph::Drawable3D(object, &drawables)
  {
    object.scale({5.0, 5.0, 5.0});
    shader_.setColor(0xffffff55_rgbaf);
    mesh_ = MeshTools::compile(Primitives::grid3DWireframe({9, 9}));
  }

  void draw(const Matrix4 & transformation, SceneGraph::Camera3D & camera) override
  {
    shader_.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation).draw(mesh_);
  }

private:
  Shaders::Flat3D shader_;
  GL::Mesh mesh_;
};

struct AxisMarker : public SceneGraph::Drawable3D
{
  AxisMarker(Object3D & object, SceneGraph::DrawableGroup3D & drawables) : SceneGraph::Drawable3D(object, &drawables)
  {
    mesh_ = MeshTools::compile(Primitives::axis3D());
  }

  void draw(const Matrix4 & transformation, SceneGraph::Camera3D & camera) override
  {
    shader_.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation).draw(mesh_);
  }

private:
  Shaders::VertexColor3D shader_;
  GL::Mesh mesh_;
};

McRtcGui::McRtcGui(const Arguments & arguments)
: Platform::Application{
      arguments,
      Configuration{}.setTitle("mc_rtc - Magnum based GUI").setWindowFlags(Configuration::WindowFlag::Resizable)}
{
  imgui_ = ImGuiIntegration::Context(Vector2{windowSize()} / dpiScaling(), windowSize(), framebufferSize());

  /* Set up proper blending to be used by ImGui. There's a great chance
     you'll need this exact behavior for the rest of your scene. If not, set
     this only for the drawFrame() call. */
  GL::Renderer::setBlendEquation(GL::Renderer::BlendEquation::Add, GL::Renderer::BlendEquation::Add);
  GL::Renderer::setBlendFunction(GL::Renderer::BlendFunction::SourceAlpha,
                                 GL::Renderer::BlendFunction::OneMinusSourceAlpha);

  colorShader_.setAmbientColor(0x111111_rgbf).setSpecularColor(0xffffff_rgbf).setShininess(80.0f);
  textureShader_.setAmbientColor(0x111111_rgbf).setSpecularColor(0x111111_rgbf).setShininess(80.0f);

  /** Plugin */
  importer_ = manager_.loadAndInstantiate("AnySceneImporter");
  loadMesh("/home/gergondet/devel/src/catkin_data_ws/src/mc_rtc_data/jvrc_description/meshes/NECK_P_S.dae");

  /** Camera setup */
  {
    const Vector3 eye{2.5f, -1.5f, 1.5f};
    const Vector3 center{0.0f, 0.0f, 0.75f};
    const Vector3 up = Vector3::zAxis();
    camera_.emplace(scene_, eye, center, up, 60.0_degf, windowSize(), framebufferSize());
  }

  root_.setParent(&scene_);

  /** Grid */
  {
    new Grid{*new Object3D{&scene_}, drawables_};
  }

  /** Axis marker */
  {
    new AxisMarker{*new Object3D{&scene_}, drawables_};
  }
}

bool McRtcGui::loadMesh(const std::string & path)
{
  if(!importer_->openFile(path))
  {
    return false;
  }
  textures_ = Containers::Array<Containers::Optional<GL::Texture2D>>{importer_->textureCount()};
  for(UnsignedInt i = 0; i < importer_->textureCount(); ++i)
  {
    Containers::Optional<Trade::TextureData> textureData = importer_->texture(i);
    if(!textureData || textureData->type() != Trade::TextureData::Type::Texture2D)
    {
      Warning{} << "Cannot load texture properties, skipping";
      continue;
    }

    Containers::Optional<Trade::ImageData2D> imageData = importer_->image2D(textureData->image());
    GL::TextureFormat format;
    if(imageData && imageData->format() == PixelFormat::RGB8Unorm)
      format = GL::TextureFormat::RGB8;
    else if(imageData && imageData->format() == PixelFormat::RGBA8Unorm)
      format = GL::TextureFormat::RGBA8;
    else
    {
      Warning{} << "Cannot load texture image, skipping";
      continue;
    }

    /* Configure the texture */
    GL::Texture2D texture;
    texture.setMagnificationFilter(textureData->magnificationFilter())
        .setMinificationFilter(textureData->minificationFilter(), textureData->mipmapFilter())
        .setWrapping(textureData->wrapping().xy())
        .setStorage(Math::log2(imageData->size().max()) + 1, format, imageData->size())
        .setSubImage(0, {}, *imageData)
        .generateMipmap();

    textures_[i] = std::move(texture);
  }
  /* Load all materials. Materials that fail to load will be NullOpt. The
   data will be stored directly in objects later, so save them only
   temporarily. */
  Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials{importer_->materialCount()};
  for(UnsignedInt i = 0; i != importer_->materialCount(); ++i)
  {
    Containers::Optional<Trade::MaterialData> materialData = importer_->material(i);
    if(!materialData || !(materialData->types() & Trade::MaterialType::Phong))
    {
      Warning{} << "Cannot load material, skipping";
      continue;
    }

    materials[i] = std::move(static_cast<Trade::PhongMaterialData &>(*materialData));
  }
  /* Load all meshes. Meshes that fail to load will be NullOpt. */
  meshes_ = Containers::Array<Containers::Optional<GL::Mesh>>{importer_->meshCount()};
  for(UnsignedInt i = 0; i != importer_->meshCount(); ++i)
  {
    Containers::Optional<Trade::MeshData> meshData = importer_->mesh(i);
    if(!meshData || !meshData->hasAttribute(Trade::MeshAttribute::Normal)
       || meshData->primitive() != MeshPrimitive::Triangles)
    {
      Warning{} << "Cannot load the mesh, skipping";
      continue;
    }

    /* Compile the mesh */
    meshes_[i] = MeshTools::compile(*meshData);
  }
  /* Load the scene */
  if(importer_->defaultScene() != -1)
  {
    Containers::Optional<Trade::SceneData> sceneData = importer_->scene(importer_->defaultScene());
    if(!sceneData)
    {
      Error{} << "Cannot load scene, exiting";
      return false;
    }
    /* Recursively add all children */
    for(UnsignedInt objectId : sceneData->children3D())
    {
      addObject(*importer_, materials, root_, objectId);
    }
  }
  else if(!meshes_.empty() && meshes_[0])
  {
    new ColoredDrawable{root_, colorShader_, *meshes_[0], 0xffffff_rgbf, drawables_};
  }
  return true;
}

void McRtcGui::addObject(Trade::AbstractImporter & importer,
                         Containers::ArrayView<const Containers::Optional<Trade::PhongMaterialData>> materials,
                         Object3D & parent,
                         UnsignedInt i)
{
  Containers::Pointer<Trade::ObjectData3D> objectData = importer.object3D(i);
  if(!objectData)
  {
    Error{} << "Cannot import object, skipping";
    return;
  }

  /* Add the object to the scene and set its transformation */
  auto * object = new Object3D{&parent};
  object->setTransformation(objectData->transformation());

  /* Add a drawable if the object has a mesh and the mesh is loaded */
  if(objectData->instanceType() == Trade::ObjectInstanceType3D::Mesh && objectData->instance() != -1
     && meshes_[objectData->instance()])
  {
    const Int materialId = static_cast<Trade::MeshObjectData3D *>(objectData.get())->material();

    /* Material not available / not loaded, use a default material */
    if(materialId == -1 || !materials[materialId])
    {
      new ColoredDrawable{*object, colorShader_, *meshes_[objectData->instance()], 0xffffff_rgbf, drawables_};
    }
    /* Textured material. If the texture failed to load, again just use a
       default colored material. */
    else if(materials[materialId]->hasAttribute(Trade::MaterialAttribute::DiffuseTexture))
    {
      Containers::Optional<GL::Texture2D> & texture = textures_[materials[materialId]->diffuseTexture()];
      if(texture)
      {
        new TexturedDrawable{*object, textureShader_, *meshes_[objectData->instance()], *texture, drawables_};
      }
      else
      {
        new ColoredDrawable{*object, colorShader_, *meshes_[objectData->instance()], 0xffffff_rgbf, drawables_};
      }
    }
    /* Color-only material */
    else
    {
      new ColoredDrawable{*object, colorShader_, *meshes_[objectData->instance()],
                          materials[materialId]->diffuseColor(), drawables_};
    }
  }

  /* Recursively add children */
  for(std::size_t id : objectData->children())
  {
    addObject(importer, materials, *object, id);
  }
}

void McRtcGui::drawEvent()
{
  GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
  GL::Renderer::enable(GL::Renderer::Feature::Blending);

  camera_->update();
  camera_->draw(drawables_);

  imgui_.newFrame();
  ImGuizmo::BeginFrame();

  /* Enable text input, if needed */
  if(ImGui::GetIO().WantTextInput && !isTextInputActive())
  {
    startTextInput();
  }
  else if(!ImGui::GetIO().WantTextInput && isTextInputActive())
  {
    stopTextInput();
  }

  ImGui::ShowDemoWindow();

  ImGuiIO & io = ImGui::GetIO();
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
  auto view = camera_->viewMatrix();
  auto projection = camera_->camera().projectionMatrix();
  float m[16];
  memcpy(&m, root_.transformationMatrix().data(), 16 * sizeof(float));
  ImGuizmo::SetID(0);
  if(ImGuizmo::Manipulate(view.data(), projection.data(), guizmoOperation_, guizmoMode_, m))
  {
    Matrix4 m16 = Matrix4::from(m);
    root_.setTransformation(m16);
  }

  /* Update application cursor */
  imgui_.updateApplicationCursor(*this);

  /* Set appropriate states. If you only draw ImGui, it is sufficient to
     just enable blending and scissor test in the constructor. */
  GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
  GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
  GL::Renderer::disable(GL::Renderer::Feature::DepthTest);

  imgui_.drawFrame();

  /* Reset state. Only needed if you want to draw something else with
     different state after. */
  GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
  GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
  GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);

  swapBuffers();
  redraw();
}

void McRtcGui::viewportEvent(ViewportEvent & event)
{
  GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

  imgui_.relayout(Vector2{event.windowSize()} / event.dpiScaling(), event.windowSize(), event.framebufferSize());

  camera_->reshape(event.windowSize(), event.framebufferSize());
}

void McRtcGui::keyPressEvent(KeyEvent & event)
{
  if(imgui_.handleKeyPressEvent(event))
  {
    return;
  }
  if(event.key() == KeyEvent::Key::Space)
  {
    if(guizmoOperation_ == ImGuizmo::TRANSLATE)
    {
      guizmoOperation_ = ImGuizmo::ROTATE;
    }
    else
    {
      guizmoOperation_ = ImGuizmo::TRANSLATE;
    }
    event.setAccepted();
    return;
  }
  if(event.key() == KeyEvent::Key::T)
  {
    if(guizmoMode_ == ImGuizmo::LOCAL)
    {
      guizmoMode_ = ImGuizmo::WORLD;
    }
    else
    {
      guizmoMode_ = ImGuizmo::LOCAL;
    }
  }
}

void McRtcGui::keyReleaseEvent(KeyEvent & event)
{
  if(imgui_.handleKeyReleaseEvent(event)) return;
}

void McRtcGui::mousePressEvent(MouseEvent & event)
{
  if(imgui_.handleMousePressEvent(event))
  {
    return;
  }
  camera_->initTransformation(event.position());
  event.setAccepted();
  redraw();
}

void McRtcGui::mouseReleaseEvent(MouseEvent & event)
{
  if(imgui_.handleMouseReleaseEvent(event))
  {
    return;
  }
}

void McRtcGui::mouseMoveEvent(MouseMoveEvent & event)
{
  if(imgui_.handleMouseMoveEvent(event))
  {
    return;
  }
  if(!event.buttons())
  {
    return;
  }
  if(event.modifiers() & MouseMoveEvent::Modifier::Shift)
  {
    camera_->translate(event.position());
  }
  else
  {
    camera_->rotate(event.position());
  }
  event.setAccepted();
  redraw();
}

void McRtcGui::mouseScrollEvent(MouseScrollEvent & event)
{
  if(imgui_.handleMouseScrollEvent(event))
  {
    /* Prevent scrolling the page */
    event.setAccepted();
    return;
  }
  const auto delta = event.offset().y();
  if(Math::abs(delta) < 1.0e-2f)
  {
    return;
  }
  camera_->zoom(delta);
  event.setAccepted();
  redraw();
}

void McRtcGui::textInputEvent(TextInputEvent & event)
{
  if(imgui_.handleTextInputEvent(event)) return;
}

MAGNUM_APPLICATION_MAIN(McRtcGui)
