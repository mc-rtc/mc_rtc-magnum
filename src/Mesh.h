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
};

struct Mesh : public CommonDrawable
{
  Mesh(Object3D * parent,
       SceneGraph::DrawableGroup3D * group,
       ImportedMesh & data,
       Shaders::PhongGL & colorShader,
       Shaders::PhongGL & textureShader,
       Color4 color);

  inline void alpha(float alpha) noexcept override
  {
    for(auto & d : drawables_) { d->alpha(alpha); }
  }

private:
  std::vector<CommonDrawable *> drawables_;

  void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override;
};

} // namespace mc_rtc::magnum
