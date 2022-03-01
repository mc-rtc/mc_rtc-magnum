#pragma once

#include "Camera.h"

#include <memory>

namespace mc_rtc::magnum
{

class CommonDrawable : public Object3D, public SceneGraph::Drawable3D
{
public:
  inline CommonDrawable(Object3D * parent, SceneGraph::DrawableGroup3D * group)
  : Object3D{parent}, SceneGraph::Drawable3D{*this, group}, group_(group)
  {
  }

  inline bool hidden() const noexcept
  {
    return hidden_;
  }

  void hidden(bool hidden) noexcept;

  virtual void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) = 0;

private:
  SceneGraph::DrawableGroup3D * group_;
  bool hidden_ = false;

  inline void draw(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) final
  {
    if(!hidden_)
    {
      draw_(transformationMatrix, camera);
    }
  }
};

class ColoredDrawable : public CommonDrawable
{
public:
  explicit ColoredDrawable(Object3D * parent,
                           SceneGraph::DrawableGroup3D * group,
                           Shaders::PhongGL & shader,
                           GL::Mesh & mesh,
                           const Color4 & color,
                           const Containers::Optional<Color4> & ambient = Containers::NullOpt);

  inline void color(const Color4 & color) noexcept
  {
    color_ = color;
  }

  inline void ambient(const Color4 & ambient) noexcept
  {
    ambient_ = ambient;
  }

  inline void colorWithAmbient(const Color4 & color) noexcept
  {
    color_ = color;
    if(color_.r() == color_.g() && color_.g() == color_.a())
    {
      ambient_ = 0x000000ff_rgbaf;
      ambient_.a() = color_.a();
    }
    else
    {
      ambient_ = Color4::fromHsv({color_.hue(), 1.0f, 0.3f}, color_.a());
    }
  }

protected:
  void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override;

  Shaders::PhongGL & shader_;
  GL::Mesh & mesh_;
  Color4 color_;
  Color4 ambient_;
};

class TexturedDrawable : public CommonDrawable
{
public:
  explicit TexturedDrawable(Object3D * object,
                            SceneGraph::DrawableGroup3D * group,
                            Shaders::PhongGL & shader,
                            GL::Mesh & mesh,
                            GL::Texture2D & texture);

private:
  void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override;

  Shaders::PhongGL & shader_;
  GL::Mesh & mesh_;
  GL::Texture2D & texture_;
};

class Sphere : public ColoredDrawable
{
public:
  explicit Sphere(Object3D * parent,
                  SceneGraph::DrawableGroup3D * group,
                  Shaders::PhongGL & shader,
                  GL::Mesh & mesh,
                  Vector3 center,
                  float radius,
                  Color4 color);

  inline void center(const Vector3 & c) noexcept
  {
    center_ = c;
    update();
  }

  inline void radius(float radius) noexcept
  {
    radius_ = radius;
    update();
  }

private:
  Vector3 center_;
  float radius_;
  void update() noexcept;
};

using SpherePtr = std::shared_ptr<Sphere>;

class Box : public ColoredDrawable
{
public:
  explicit Box(Object3D * parent,
               SceneGraph::DrawableGroup3D * group,
               Shaders::PhongGL & shader,
               GL::Mesh & mesh,
               Matrix4 pose,
               Vector3 size,
               Color4 color);

  inline void pose(const Matrix4 & pose) noexcept
  {
    pose_ = pose;
    update();
  }

  inline void size(const Vector3 & size) noexcept
  {
    size_ = size;
    update();
  }

private:
  Matrix4 pose_;
  Vector3 size_;
  void update() noexcept;
};

using BoxPtr = std::shared_ptr<Box>;

} // namespace mc_rtc::magnum
