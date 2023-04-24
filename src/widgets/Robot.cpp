#include "Robot.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/version.h>

namespace mc_rtc::magnum
{

namespace details
{

template<typename T>
void setConfiguration(T & robot, const std::vector<std::vector<double>> & q)
{
  static_assert(std::is_same_v<T, mc_rbdyn::Robot>);
  robot.mbc().q = q;
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

struct RobotCache
{
  inline static std::shared_ptr<mc_rbdyn::Robots> get_robot(const std::vector<std::string> & params)
  {
    if(robots_.count(params))
    {
      use_cnt_[params] += 1;
    }
    else
    {
      use_cnt_[params] = 1;
      robots_[params] = mc_rbdyn::loadRobot(*fromParams(params));
    }
    auto out = mc_rbdyn::Robots::make([](mc_rbdyn::Robots * robots) {
      remove_robot(robots->robot().module().parameters());
      delete robots;
    });
    auto & robot = robots_[params]->robot();
    out->robotCopy(robot, robot.name());
    return out;
  }

  inline static void remove_robot(const std::vector<std::string> & params)
  {
    use_cnt_[params] -= 1;
    if(use_cnt_[params] == 0)
    {
      use_cnt_.erase(params);
      robots_.erase(params);
    }
  }

private:
  inline static std::map<std::vector<std::string>, std::shared_ptr<mc_rbdyn::Robots>> robots_ = {};
  inline static std::map<std::vector<std::string>, size_t> use_cnt_ = {};
};

struct RobotImpl
{
  RobotImpl(Robot & robot) : self_(robot)
  {
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
    return self_.gui();
  }

  void data(const std::vector<std::string> & params,
            const std::vector<std::vector<double>> & q,
            const sva::PTransformd & posW)
  {
    if(!robots_ || robot().module().parameters() != params)
    {
      visualObjects_.clear();
      collisionObjects_.clear();
      robots_ = RobotCache::get_robot(params);
      const auto & rm = robots_->robot().module();
      using Objects = std::vector<std::shared_ptr<CommonDrawable>>;
      auto loadMeshCallback = [&](Objects & objects, std::vector<std::function<void(float)>> & draws, size_t bIdx,
                                  const rbd::parsers::Visual & visual, bool hidden) {
        const auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(visual.geometry.data);
        auto path = convertURI(mesh.filename, rm.path);
        auto object = gui().loadMesh(path.string(), color(visual.material));
        auto scale = static_cast<float>(mesh.scale);
        object->hidden(hidden);
        objects.push_back(object);
        draws.push_back([this, bIdx, visual, object, scale](float alpha) {
          const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
          object->alpha(alpha);
          object->setTransformation(convert(X_0_b) * Matrix4::scaling({scale, scale, scale}));
        });
      };
      auto loadBoxCallback = [&](Objects & objects, std::vector<std::function<void(float)>> & draws, size_t bIdx,
                                 const rbd::parsers::Visual & visual, bool hidden) {
        const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
        const auto & box = boost::get<rbd::parsers::Geometry::Box>(visual.geometry.data);
        auto object =
            gui().makeBox(translation(X_0_b), convert(X_0_b.rotation()), translation(box.size), color(visual.material));
        object->hidden(hidden);
        objects.push_back(object);
        draws.push_back([this, bIdx, visual, object](float alpha) {
          const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
          auto & box = static_cast<Box &>(*object);
          box.alpha(alpha);
          box.pose(convert(X_0_b));
        });
      };
      auto loadCylinderCallback = [&](std::vector<std::function<void(float)>> & draws, size_t bIdx,
                                      const rbd::parsers::Visual & visual) {
        draws.push_back([this, bIdx, visual](float alpha) {
          const auto & cylinder = boost::get<rbd::parsers::Geometry::Cylinder>(visual.geometry.data);
          const auto & start = sva::PTransformd(Eigen::Vector3d{0.0, 0.0, -cylinder.length / 2}) * visual.origin
                               * robot().mbc().bodyPosW[bIdx];
          const auto & end = sva::PTransformd(Eigen::Vector3d{0.0, 0.0, cylinder.length}) * start;
          gui().drawArrow(translation(start), translation(end), 2 * cylinder.radius, 0.0f, 0.0f,
                          Color4(color(visual.material).rgb(), alpha));
        });
      };
      auto loadSphereCallback = [&](Objects & objects, std::vector<std::function<void(float)>> & draws, size_t bIdx,
                                    const rbd::parsers::Visual & visual, bool hidden) {
        const auto & sphere = boost::get<rbd::parsers::Geometry::Sphere>(visual.geometry.data);
        auto s = gui().makeSphere({}, static_cast<float>(sphere.radius), color(visual.material));
        s->hidden(hidden);
        objects.push_back(s);
        draws.push_back([this, bIdx, visual, s](float alpha) {
          const auto & X_0_b = visual.origin * robot().mbc().bodyPosW[bIdx];
          s->alpha(alpha);
          s->center(translation(X_0_b));
        });
      };
      auto loadBodyCallbacks = [&](Objects & objects, std::vector<std::function<void(float)>> & draws, size_t bIdx,
                                   const std::vector<rbd::parsers::Visual> & visuals, bool hidden) {
        for(const auto & visual : visuals)
        {
          using Geometry = rbd::parsers::Geometry;
          switch(visual.geometry.type)
          {
            case Geometry::MESH:
              loadMeshCallback(objects, draws, bIdx, visual, hidden);
              break;
            case Geometry::BOX:
              loadBoxCallback(objects, draws, bIdx, visual, hidden);
              break;
            case Geometry::CYLINDER:
              loadCylinderCallback(draws, bIdx, visual);
              break;
            case Geometry::SPHERE:
              loadSphereCallback(objects, draws, bIdx, visual, hidden);
              break;
            default:
              break;
          };
        }
      };
      auto loadCallbacks = [&](Objects & objects, std::vector<std::function<void(float)>> & draws, const auto & visuals,
                               bool hidden) {
        draws.clear();
        const auto & bodies = robot().mb().bodies();
        for(size_t i = 0; i < bodies.size(); ++i)
        {
          const auto & b = bodies[i];
          if(!visuals.count(b.name()))
          {
            continue;
          }
          loadBodyCallbacks(objects, draws, i, visuals.at(b.name()), hidden);
        }
      };
      loadCallbacks(visualObjects_, drawVisual_, robot().module()._visual, !drawVisualModel_);
      loadCallbacks(collisionObjects_, drawCollision_, robot().module()._collision, !drawCollisionModel_);
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
    ImGui::Columns(2);
    if(ImGui::Checkbox(self_.label(fmt::format("Draw {} visual model", self_.id.name)).c_str(), &drawVisualModel_))
    {
      for(auto & o : visualObjects_)
      {
        o->hidden(!drawVisualModel_);
      }
    }
    ImGui::NextColumn();
    ImGui::SliderFloat(self_.label("Alpha##Visual", self_.id.name).c_str(), &drawVisualModelAlpha_, 0.0, 1.0);
    ImGui::NextColumn();
    if(ImGui::Checkbox(self_.label(fmt::format("Draw {} collision model", self_.id.name)).c_str(),
                       &drawCollisionModel_))
    {
      for(auto & o : collisionObjects_)
      {
        o->hidden(!drawCollisionModel_);
      }
    }
    ImGui::NextColumn();
    ImGui::SliderFloat(self_.label("Alpha##Collision", self_.id.name).c_str(), &drawCollisionModelAlpha_, 0.0, 1.0);
    ImGui::Columns(1);
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
        d(drawVisualModelAlpha_);
      }
    }
    if(drawCollisionModel_)
    {
      for(const auto & d : drawCollision_)
      {
        d(drawCollisionModelAlpha_);
      }
    }
  }

private:
  Robot & self_;
  std::shared_ptr<mc_rbdyn::Robots> robots_;
  bool drawVisualModel_ = true;
  float drawVisualModelAlpha_ = 1.0;
  bool drawCollisionModel_ = false;
  float drawCollisionModelAlpha_ = 1.0;
  std::vector<std::shared_ptr<CommonDrawable>> visualObjects_;
  std::vector<std::function<void(float alpha)>> drawVisual_;
  std::vector<std::shared_ptr<CommonDrawable>> collisionObjects_;
  std::vector<std::function<void(float alpha)>> drawCollision_;
};

} // namespace details

Robot::Robot(Client & client, const ElementId & id, McRtcGui & gui)
: Widget(client, id, gui), impl_(new details::RobotImpl{*this})
{
}

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

} // namespace mc_rtc::magnum
