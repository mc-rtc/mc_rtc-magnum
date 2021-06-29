#pragma once

#include "Widget.h"

namespace mc_rtc::magnum
{

struct Visual : public Widget
{
  Visual(Client & client, const ElementId & id);

  void data(const rbd::parsers::Visual & visual, const sva::PTransformd & pos);

  void draw2D() override;

  void draw3D() override;

private:
  rbd::parsers::Visual visual_;
  bool typeChanged_ = false;
  sva::PTransformd pos_;
  std::shared_ptr<CommonDrawable> object_;
  bfs::path mesh_;
};

} // namespace mc_rtc::magnum
