#include "Polyhedron.h"

namespace mc_rtc::magnum
{

void Polyhedron::data(const std::vector<Eigen::Vector3d> & vertices,
                      const std::vector<std::array<size_t, 3>> & triangles,
                      const std::vector<mc_rtc::gui::Color> & colors,
                      const mc_rtc::gui::PolyhedronConfig & config)
{
  if(!poly_) { poly_ = gui_.makePolyhedron(); }
  poly_->update(vertices, triangles, colors, config);
}

void Polyhedron::draw2D()
{
  if(poly_)
  {
    bool show = !poly_->hidden();
    if(ImGui::Checkbox(label(fmt::format("Show {}", id.name)).c_str(), &show)) { poly_->hidden(!show); }
  }
}

} // namespace mc_rtc::magnum
