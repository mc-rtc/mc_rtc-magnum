#include "Robot.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/version.h>

namespace details
{

#ifndef MC_RTC_VERSION_MAJOR
static constexpr int MC_RTC_VERSION_MAJOR = mc_rtc::MC_RTC_VERSION[0] - '0';
#endif

template<typename T>
void setConfiguration(T & robot, const std::vector<std::vector<double>> & q)
{
  static_assert(std::is_same_v<T, mc_rbdyn::Robot>);
  if constexpr(MC_RTC_VERSION_MAJOR > 1)
  {
    robot.q()->set(rbd::paramToVector(robot.mb(), q));
  }
  else
  {
    robot.mbc().q = q;
  }
}

inline mc_rbdyn::RobotModulePtr fromParams(const std::vector<std::string> & p)
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

struct RobotImpl
{
  RobotImpl(Robot & robot) : self_(robot)
  {
    collisionRoot_.setParent(&gui().scene());
    visualRoot_.setParent(&gui().scene());
    if(self_.id.category.size() > 1)
    {
      drawVisualModel_ = false;
    }
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
          const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
          object->setTransformation(convert(X_0_b));
        });
      };
      auto loadBoxCallback = [&](std::vector<std::function<void()>> & draws, size_t bIdx,
                                 const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual]() {
          const auto & box = boost::get<rbd::parsers::Geometry::Box>(visual.geometry.data);
          const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
          gui().drawCube(translation(X_0_b), convert(X_0_b.rotation()), translation(box.size), color(visual.material));
        });
      };
      auto loadCylinderCallback = [&](std::vector<std::function<void()>> & draws, size_t bIdx,
                                      const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual]() {
          const auto & cylinder = boost::get<rbd::parsers::Geometry::Cylinder>(visual.geometry.data);
          const auto & start = sva::PTransformd(Eigen::Vector3d{0.0, 0.0, -cylinder.length / 2}) * visual.origin
                               * robot().mbc().bodyPosW[bIdx];
          const auto & end = sva::PTransformd(Eigen::Vector3d{0.0, 0.0, cylinder.length}) * start;
          gui().drawArrow(translation(start), translation(end), 2 * cylinder.radius, 0.0f, 0.0f,
                          color(visual.material));
        });
      };
      auto loadSphereCallback = [&](std::vector<std::function<void()>> & draws, size_t bIdx,
                                    const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual]() {
          const auto & sphere = boost::get<rbd::parsers::Geometry::Sphere>(visual.geometry.data);
          const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
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
                               SceneGraph::DrawableGroup3D & group, const auto & visuals) {
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
    setConfiguration(robot(), q);
    robot().posW(posW);
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
      gui().camera().camera()->draw(visualGroup_);
    }
    if(drawCollisionModel_)
    {
      for(const auto & d : drawCollision_)
      {
        d();
      }
      gui().camera().camera()->draw(collisionGroup_);
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
