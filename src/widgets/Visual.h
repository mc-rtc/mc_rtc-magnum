#pragma once

#include "Widget.h"

struct Visual : public Widget
{
  Visual(Client & client, const ElementId & id);

  void data(const rbd::parsers::Visual & visual,
            const sva::PTransformd & pos);

  void draw2D() override {}

  void draw3D() override;
private:
  rbd::parsers::Visual visual_;
  sva::PTransformd pos_;
  std::unique_ptr<Object3D> object_;
  SceneGraph::DrawableGroup3D group_;
  bfs::path mesh_;
};
