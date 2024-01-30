#include "Polyhedron.h"

namespace mc_rtc::magnum
{

void Polyhedron::data(const std::vector<Eigen::Vector3d> & vertices,
                      const std::vector<std::array<size_t, 3>> & triangles,
                      const std::vector<mc_rtc::gui::Color> & colors,
                      const mc_rtc::gui::PolyhedronConfig & config)
{
  if(!poly_)
  {
    poly_ = gui_.makePolyhedron();
    poly_->draw_wireframe(config.show_edges);
  }
  poly_->update(vertices, triangles, colors, config);
}

void Polyhedron::draw2D()
{
  if(poly_)
  {
    bool show = !poly_->hidden();
    if(ImGui::Checkbox(label(fmt::format("Show {}", id.name)).c_str(), &show)) { poly_->hidden(!show); }
    bool show_wire = poly_->draw_wireframe();
    ImGui::SameLine();
    if(ImGui::Checkbox(label(fmt::format("Show {} wireframe", id.name)).c_str(), &show_wire))
    {
      poly_->draw_wireframe(show_wire);
    }
  }
}

} // namespace mc_rtc::magnum
