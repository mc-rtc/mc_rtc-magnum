#pragma once

#include "Widget.h"

namespace mc_rtc::magnum
{

struct Polyhedron : public Widget
{
  using Widget::Widget;

  void data(const std::vector<Eigen::Vector3d> & vertices,
            const std::vector<std::array<size_t, 3>> & triangles,
            const std::vector<mc_rtc::gui::Color> & colors,
            const mc_rtc::gui::PolyhedronConfig & config);

  void draw2D() override;

private:
  PolyhedronPtr poly_;
  float alpha_ = 1.0f;
};

} // namespace mc_rtc::magnum
