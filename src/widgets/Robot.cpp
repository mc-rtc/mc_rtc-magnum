#include "Robot.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#ifdef MC_RTC_HAS_ROS_SUPPORT
#  include <ros/package.h>
#endif

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/config.h>

namespace details
{

inline bfs::path convertURI(const std::string & uri)
{
  const std::string package = "package://";
  if(uri.size() >= package.size() && uri.find(package) == 0)
  {
    size_t split = uri.find('/', package.size());
    std::string pkg = uri.substr(package.size(), split - package.size());
    auto leaf = bfs::path(uri.substr(split + 1));
    bfs::path MC_ENV_DESCRIPTION_PATH(mc_rtc::MC_ENV_DESCRIPTION_PATH);
#ifndef __EMSCRIPTEN__
#  ifndef MC_RTC_HAS_ROS_SUPPORT
    // FIXME Prompt the user for unknown packages
    if(pkg == "jvrc_description")
    {
      pkg = (MC_ENV_DESCRIPTION_PATH / ".." / "jvrc_description").string();
    }
    else if(pkg == "mc_env_description")
    {
      pkg = MC_ENV_DESCRIPTION_PATH.string();
    }
    else if(pkg == "mc_int_obj_description")
    {
      pkg = (MC_ENV_DESCRIPTION_PATH / ".." / "mc_int_obj_description").string();
    }
    else
    {
      mc_rtc::log::critical("Cannot resolve package: {}", pkg);
    }
#  else
    pkg = ros::package::getPath(pkg);
#  endif
#else
    pkg = "/assets/" + pkg;
#endif
    return pkg / leaf;
  }
  const std::string file = "file://";
  if(uri.size() >= file.size() && uri.find(file) == 0)
  {
    return bfs::path(uri.substr(file.size()));
  }
  return uri;
}

mc_rbdyn::RobotModulePtr fromParams(const std::vector<std::string> & p)
{
  mc_rbdyn::RobotModulePtr rm{nullptr};
  if(p.size() == 1)
  {
    rm = mc_rbdyn::RobotLoader::get_robot_module(p[0]);
  }
  if(p.size() == 2)
  {
    rm = mc_rbdyn::RobotLoader::get_robot_module(p[0], p[1]);
  }
  if(p.size() == 3)
  {
    rm = mc_rbdyn::RobotLoader::get_robot_module(p[0], p[1], p[2]);
  }
  if(p.size() > 3)
  {
    mc_rtc::log::warning("Too many parameters provided to load the robot, complain to the developpers of this package");
  }
  return rm;
}

inline Magnum::Color4 color(const rbd::parsers::Material & m)
{
  if(m.type == rbd::parsers::Material::Type::COLOR)
  {
    const auto & c = boost::get<rbd::parsers::Material::Color>(m.data);
    return {static_cast<float>(c.r), static_cast<float>(c.g), static_cast<float>(c.b), static_cast<float>(c.a)};
  }
  return {1.0f, 1.0f, 1.0f, 1.0f};
}

struct RobotImpl
{
  RobotImpl(Robot & robot) : self_(robot)
  {
    collisionRoot_.setParent(&gui().scene());
    visualRoot_.setParent(&gui().scene());
  }

  inline mc_rbdyn::Robot & robot()
  {
    return robots_->robot();
  }

  inline McRtcGui & gui()
  {
    return self_.client.gui();
  }

  void data(const std::vector<std::string> & params,
            const std::vector<std::vector<double>> & q,
            const sva::PTransformd & posW)
  {
    if(!robots_ || robot().module().parameters() != params)
    {
      auto rm = fromParams(params);
      if(!rm)
      {
        return;
      }
      robots_ = mc_rbdyn::loadRobot(*rm);
      auto loadMeshCallback = [&](std::vector<std::function<void()>> & draws, Object3D & root,
                                  SceneGraph::DrawableGroup3D & group, size_t bIdx,
                                  const rbd::parsers::Visual & visual) {
        const auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(visual.geometry.data);
        auto path = convertURI(mesh.filename);
        auto object = std::make_shared<Object3D>(&root);
        gui().loadMesh(path.string(), *object, group);
        draws.push_back([this, bIdx, visual, object]() {
          const auto & X_0_b = visual.origin * robot().bodyPosW()[bIdx];
          object->setTransformation(convert(X_0_b));
        });
      };
      auto loadBoxCallback = [&](std::vector<std::function<void()>> & draws, size_t bIdx,
                                 const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual]() {
          const auto & box = boost::get<rbd::parsers::Geometry::Box>(visual.geometry.data);
          const auto & X_0_b = visual.origin * robot().bodyPosW()[bIdx];
          gui().drawCube(translation(X_0_b), convert(X_0_b.rotation()), translation(box.size), color(visual.material));
        });
      };
      auto loadCylinderCallback = [&](std::vector<std::function<void()>> & draws, size_t bIdx,
                                      const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual]() {
          const auto & cylinder = boost::get<rbd::parsers::Geometry::Cylinder>(visual.geometry.data);
          const auto & X_0_b = visual.origin * robot().bodyPosW()[bIdx];
          // FIXME
        });
      };
      auto loadSphereCallback = [&](std::vector<std::function<void()>> & draws, size_t bIdx,
                                    const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual]() {
          const auto & sphere = boost::get<rbd::parsers::Geometry::Sphere>(visual.geometry.data);
          const auto & X_0_b = visual.origin * robot().bodyPosW()[bIdx];
          gui().drawSphere(translation(X_0_b), static_cast<float>(sphere.radius), color(visual.material));
        });
      };
      auto loadBodyCallbacks = [&](std::vector<std::function<void()>> & draws, Object3D & root,
                                   SceneGraph::DrawableGroup3D & group, size_t bIdx,
                                   const std::vector<rbd::parsers::Visual> & visuals) {
        for(const auto & visual : visuals)
        {
          using Geometry = rbd::parsers::Geometry;
          switch(visual.geometry.type)
          {
            case Geometry::MESH:
              loadMeshCallback(draws, root, group, bIdx, visual);
              break;
            case Geometry::BOX:
              loadBoxCallback(draws, bIdx, visual);
              break;
            case Geometry::CYLINDER:
              loadCylinderCallback(draws, bIdx, visual);
              break;
            case Geometry::SPHERE:
              loadSphereCallback(draws, bIdx, visual);
              break;
            default:
              break;
          };
        }
      };
      auto loadCallbacks = [&](std::vector<std::function<void()>> & draws, Object3D & root,
                               SceneGraph::DrawableGroup3D & group,
                               const std::map<std::string, std::vector<rbd::parsers::Visual>> & visuals) {
        draws.clear();
        const auto & bodies = robot().mb().bodies();
        for(size_t i = 0; i < bodies.size(); ++i)
        {
          const auto & b = bodies[i];
          if(!visuals.count(b.name()))
          {
            continue;
          }
          loadBodyCallbacks(draws, root, group, i, visuals.at(b.name()));
        }
      };
      loadCallbacks(drawVisual_, visualRoot_, visualGroup_, robot().module()._visual);
      loadCallbacks(drawCollision_, collisionRoot_, collisionGroup_, robot().module()._collision);
    }
    robot().posW(posW);
    robot().mbc().q = q;
  }

  void draw2D()
  {
    if(!robots_)
    {
      return;
    }
    ImGui::Checkbox(self_.label(fmt::format("Draw {} visual model", self_.id.name)).c_str(), &drawVisualModel_);
    ImGui::Checkbox(self_.label(fmt::format("Draw {} collision model", self_.id.name)).c_str(), &drawCollisionModel_);
  }

  void draw3D()
  {
    if(!robots_)
    {
      return;
    }
    if(drawVisualModel_)
    {
      for(const auto & d : drawVisual_)
      {
        d();
      }
      gui().camera().camera().draw(visualGroup_);
    }
    if(drawCollisionModel_)
    {
      for(const auto & d : drawCollision_)
      {
        d();
      }
      gui().camera().camera().draw(collisionGroup_);
    }
  }

private:
  Robot & self_;
  std::shared_ptr<mc_rbdyn::Robots> robots_;
  bool drawVisualModel_ = true;
  bool drawCollisionModel_ = false;
  Object3D visualRoot_;
  SceneGraph::DrawableGroup3D visualGroup_;
  std::vector<std::function<void()>> drawVisual_;
  Object3D collisionRoot_;
  SceneGraph::DrawableGroup3D collisionGroup_;
  std::vector<std::function<void()>> drawCollision_;
};

} // namespace details

Robot::Robot(Client & client, const ElementId & id) : Widget(client, id), impl_(new details::RobotImpl{*this}) {}

Robot::~Robot() = default;

void Robot::data(const std::vector<std::string> & params,
                 const std::vector<std::vector<double>> & q,
                 const sva::PTransformd & posW)
{
  impl_->data(params, q, posW);
}

void Robot::draw2D()
{
  impl_->draw2D();
}

void Robot::draw3D()
{
  impl_->draw3D();
}
