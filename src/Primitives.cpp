#include "Primitives.h"

namespace mc_rtc::magnum
{

void CommonDrawable::hidden(bool hidden) noexcept
{
  if(hidden_ == hidden)
  {
    return;
  }
  hidden_ = hidden;
  if(hidden)
  {
    group_->remove(*this);
  }
  else
  {
    group_->add(*this);
  }
  for(auto & c : children())
  {
    static_cast<CommonDrawable &>(c).hidden(hidden);
  }
}

ColoredDrawable::ColoredDrawable(Object3D * object,
                                 SceneGraph::DrawableGroup3D * group,
                                 Shaders::Phong & shader,
                                 GL::Mesh & mesh,
                                 const Color4 & color)
: CommonDrawable(object, group), shader_(shader), mesh_(mesh), color_(color)
{
}

void ColoredDrawable::draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera)
{
  Color4 ambient_;
  if(color_.r() == color_.g() && color_.g() == color_.b())
  {
    // Gray
    ambient_ = 0x000000ff_rgbaf;
    ambient_.a() = color_.a();
  }
  else
  {
    ambient_ = Color4::fromHsv({color_.hue(), 1.0f, 0.3f}, color_.a());
  }
  shader_.setDiffuseColor(color_)
      .setAmbientColor(ambient_)
      .setTransformationMatrix(transformationMatrix)
      .setNormalMatrix(transformationMatrix.normalMatrix())
      .setProjectionMatrix(camera.projectionMatrix())
      .draw(mesh_);
}

TexturedDrawable::TexturedDrawable(Object3D * object,
                                   SceneGraph::DrawableGroup3D * group,
                                   Shaders::Phong & shader,
                                   GL::Mesh & mesh,
                                   GL::Texture2D & texture)
: CommonDrawable{object, group}, shader_(shader), mesh_(mesh), texture_(texture)
{
}

void TexturedDrawable::draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera)
{
  shader_.setTransformationMatrix(transformationMatrix)
      .setNormalMatrix(transformationMatrix.normalMatrix())
      .setProjectionMatrix(camera.projectionMatrix())
      .bindDiffuseTexture(texture_)
      .draw(mesh_);
}

} // namespace mc_rtc::magnum
