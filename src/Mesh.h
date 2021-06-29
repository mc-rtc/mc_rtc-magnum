#pragma once

#include "Primitives.h"

#include <memory>

namespace mc_rtc::magnum
{

struct ImportedMesh
{
  Containers::Array<Containers::Optional<GL::Mesh>> meshes_;
  Containers::Array<Containers::Optional<Trade::PhongMaterialData>> materials_;
  Containers::Array<Containers::Optional<GL::Texture2D>> textures_;
  Containers::Optional<Trade::SceneData> scene_;
  Containers::Array<Containers::Pointer<Trade::ObjectData3D>> objects_;
};

struct Mesh : public CommonDrawable
{
  Mesh(Object3D * parent,
       SceneGraph::DrawableGroup3D * group,
       ImportedMesh & data,
       Shaders::Phong & colorShader,
       Shaders::Phong & textureShader,
       Color4 color);

private:
  std::vector<CommonDrawable *> drawables_;

  void addObject(Object3D * parent,
                 SceneGraph::DrawableGroup3D * group,
                 ImportedMesh & data,
                 UnsignedInt idx,
                 Shaders::Phong & colorShader,
                 Shaders::Phong & textureShader,
                 Color4 color);

  void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override;
};

} // namespace mc_rtc::magnum
