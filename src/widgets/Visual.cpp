#include "Visual.h"

Visual::Visual(Client & client, const ElementId & id)
: Widget(client, id)
{
}

void Visual::data(const rbd::parsers::Visual & visual,
                  const sva::PTransformd & pos)
{
  visual_ = visual;
  // Bake the visual origin in the position
  pos_ = visual.origin * pos;
}

void Visual::draw3D()
{
  using Geometry = rbd::parsers::Geometry;
  using Type = rbd::parsers::Geometry::Type;
  auto handleMesh = [&]() {
    const auto & in = boost::get<Geometry::Mesh>(visual_.geometry.data);
    auto path = convertURI(in.filename);
    if(path != mesh_)
    {
      mesh_ = path;
      object_ = std::make_unique<Object3D>(&client.gui().scene());
      client.gui().loadMesh(path.string(), *object_, group_);
    }
    object_->setTransformation(convert(pos_));
    client.gui().camera().camera()->draw(group_);
  };
  auto handleBox = [&]() {
    const auto & box = boost::get<Geometry::Box>(visual_.geometry.data);
    client.gui().drawCube(translation(pos_), rotation(pos_), translation(box.size), color(visual_.material));
  };
  auto handleCylinder = [&]() {
    const auto & cyl = boost::get<Geometry::Cylinder>(visual_.geometry.data);
    const auto & start = sva::PTransformd{Eigen::Vector3d{0, 0, -cyl.length / 2}} * pos_;
    const auto & end = sva::PTransformd{Eigen::Vector3d{0, 0, cyl.length / 2}} * pos_;
    client.gui().drawArrow(translation(start), translation(end), 2 * cyl.radius, 0.0f, 0.0f, color(visual_.material));
  };
  auto handleSphere = [&]() {
    const auto & sphere = boost::get<rbd::parsers::Geometry::Sphere>(visual_.geometry.data);
    client.gui().drawSphere(translation(pos_), static_cast<float>(sphere.radius), color(visual_.material));
  };
  switch(visual_.geometry.type)
  {
    case Type::MESH:
      handleMesh();
      break;
    case Type::BOX:
      handleBox();
      break;
    case Type::CYLINDER:
      handleCylinder();
      break;
    case Type::SPHERE:
      handleSphere();
      break;
    default:
      break;
  }
}
