#include "Polygon.h"

#include <Corrade/Containers/GrowableArray.h>

#include <Magnum/MeshTools/CompileLines.h>
#include <Magnum/MeshTools/GenerateLines.h>

namespace mc_rtc::magnum
{

namespace
{

constexpr Trade::MeshAttributeData AttributeData3DWireframe[]{
    Trade::MeshAttributeData{Trade::MeshAttribute::Position, VertexFormat::Vector3, 0, 0, sizeof(Vector3)}};

}

void Polygon::data(const std::vector<std::vector<Eigen::Vector3d>> & points, const mc_rtc::gui::LineConfig & config)
{
  if(points_ != points)
  {
    points_ = points;
    polygons_.resize(points_.size());
    for(size_t i = 0; i < points_.size(); ++i)
    {
      const auto & ps = points[i];
      auto & poly = polygons_[i];
      if(ps.size() <= 1)
      {
        poly.mesh = std::nullopt;
        continue;
      }
      Containers::Array<char> vertexData(ps.size() * sizeof(Vector3));
      auto positions = Containers::arrayCast<Vector3>(vertexData);
      for(size_t j = 0; j < ps.size(); ++j) { positions[j] = translation(ps[j]); }
      Trade::MeshData data{MeshPrimitive::LineLoop, std::move(vertexData),
                           Trade::meshAttributeDataNonOwningArray(AttributeData3DWireframe),
                           UnsignedInt(positions.size())};
      poly.mesh = MeshTools::compileLines(MeshTools::generateLines(data));
    }
  }
  config_ = config;
}

void Polygon::draw3D()
{
  if(points_.empty()) { return; }
  Color4 c = convert(config_.color);
  // This scaling seems to give a nice equivalent to RViZ
  float width = 200.0f * static_cast<float>(config_.width);
  auto & camera = *gui_.camera().camera();
  for(auto & p : polygons_)
  {
    if(!p.mesh) { continue; }
    lineShader_.setViewportSize(Vector2{GL::defaultFramebuffer.viewport().size()})
        .setTransformationProjectionMatrix(camera.projectionMatrix() * camera.cameraMatrix())
        .setColor(c)
        .setWidth(width)
        .setSmoothness(1.0f)
        .draw(*p.mesh);
  }
}

} // namespace mc_rtc::magnum
