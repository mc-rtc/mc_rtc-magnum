#pragma once

#include "Camera.h"

namespace mc_rtc::magnum
{

class CommonDrawable : public Object3D, public SceneGraph::Drawable3D
{
public:
  inline CommonDrawable(Object3D * parent, SceneGraph::DrawableGroup3D * group)
  : Object3D{parent}, SceneGraph::Drawable3D{*this, group}, group_(group)
  {
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
                           Shaders::Phong & shader,
                           GL::Mesh & mesh,
                           const Color4 & color);

  inline void color(const Color4 & color) noexcept
  {
    color_ = color;
  }

private:
  void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override;

  Shaders::Phong & shader_;
  GL::Mesh & mesh_;
  Color4 color_;
};

class TexturedDrawable : public CommonDrawable
{
public:
  explicit TexturedDrawable(Object3D * object,
                            SceneGraph::DrawableGroup3D * group,
                            Shaders::Phong & shader,
                            GL::Mesh & mesh,
                            GL::Texture2D & texture);

private:
  void draw_(const Matrix4 & transformationMatrix, SceneGraph::Camera3D & camera) override;

  Shaders::Phong & shader_;
  GL::Mesh & mesh_;
  GL::Texture2D & texture_;
};

} // namespace mc_rtc::magnum
