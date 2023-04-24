#include "Primitives.h"

namespace mc_rtc::magnum
{

static void set_children_hidden(Object3D * object, bool hidden)
{
  for(auto & c : object->children())
  {
    auto * cd = dynamic_cast<CommonDrawable *>(&c);
    if(cd)
    {
      cd->hidden(hidden);
    }
    else
    {
      set_children_hidden(&c, hidden);
    }
  }
}

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
  set_children_hidden(this, hidden);
}

ColoredDrawable::ColoredDrawable(Object3D * object,
                                 SceneGraph::DrawableGroup3D * group,
                                 Shaders::PhongGL & shader,
                                 GL::Mesh & mesh,
                                 const Color4 & color,
                                 const Containers::Optional<Color4> & ambient)
: CommonDrawable(object, group), shader_(shader), mesh_(mesh), color_(color)
{
  if(ambient)
  {
    ambient_ = *ambient;
  }
  else
  {
    colorWithAmbient(color_);
  }
  ambient_.a() = 0.0f;
}

void ColoredDrawable::draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera)
{
  shader_.setDiffuseColor(Color4(color_))
      .setAmbientColor(ambient_)
      .setTransformationMatrix(transformationMatrix)
      .setNormalMatrix(transformationMatrix.normalMatrix())
      .setProjectionMatrix(camera.projectionMatrix())
      .draw(mesh_);
}

TexturedDrawable::TexturedDrawable(Object3D * object,
                                   SceneGraph::DrawableGroup3D * group,
                                   Shaders::PhongGL & shader,
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
      .setDiffuseColor(Color4(1.0, 1.0, 1.0, alpha_))
      .draw(mesh_);
}

Sphere::Sphere(Object3D * parent,
               SceneGraph::DrawableGroup3D * group,
               Shaders::PhongGL & shader,
               GL::Mesh & mesh,
               Vector3 center,
               float radius,
               Color4 color)
: ColoredDrawable(parent, group, shader, mesh, color), center_(center), radius_(radius)
{
  update();
}

void Sphere::update() noexcept
{
  setTransformation(Matrix4::from(Matrix3{Math::IdentityInit, radius_}, center_));
}

Box::Box(Object3D * parent,
         SceneGraph::DrawableGroup3D * group,
         Shaders::PhongGL & shader,
         GL::Mesh & mesh,
         Matrix4 pose,
         Vector3 size,
         Color4 color)
: ColoredDrawable(parent, group, shader, mesh, color), pose_(pose), size_(size)
{
  update();
}

void Box::update() noexcept
{
  setTransformation(pose_ * Matrix4::scaling(size_ / 2.0));
}

} // namespace mc_rtc::magnum
