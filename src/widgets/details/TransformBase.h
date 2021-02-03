#pragma once

#include "../Widget.h"

#include "../../InteractiveMarker.h"

template<ControlAxis ctl>
struct TransformBase : public Widget
{
  TransformBase(Client & client, const ElementId & id, const ElementId & requestId)
  : Widget(client, id), requestId_(requestId)
  {
  }

  ~TransformBase() override = default;

  void data(bool ro, const sva::PTransformd & pos)
  {
    pos_ = pos;
    if(!ro && !marker_)
    {
      marker_ = std::make_unique<InteractiveMarker>(pos_, ctl);
    }
    if(ro && marker_)
    {
      marker_.reset(nullptr);
    }
    if(marker_ && !marker_->active())
    {
      marker_->pose(pos_);
    }
  }

  void update(SceneState & state) override
  {
    if(marker_)
    {
      marker_->update(state);
      pos_ = marker_->pose();
      if(marker_->active())
      {
        if constexpr(ctl == ControlAxis::TRANSLATION)
        {
          client.send_request(requestId_, pos_.translation());
        }
        else if constexpr(ctl == ControlAxis::ROTATION)
        {
          client.send_request(requestId_, pos_.rotation());
        }
        else if constexpr(ctl == ControlAxis::ALL)
        {
          client.send_request(requestId_, pos_);
        }
        else if constexpr(ctl == ControlAxis::XYTHETA || ctl == ControlAxis::XYZTHETA)
        {
          Eigen::VectorXd data = Eigen::VectorXd::Zero(4);
          const auto & t = pos_.translation();
          auto yaw = mc_rbdyn::rpyFromMat(pos_.rotation()).z();
          data(0) = t.x();
          data(1) = t.y();
          data(2) = yaw;
          data(3) = t.z();
          client.send_request(requestId_, data);
        }
      }
    }
  }

  void draw3D(Camera) override
  {
    if(marker_)
    {
      marker_->draw();
    }
  }

protected:
  ElementId requestId_;
  sva::PTransformd pos_;
  std::unique_ptr<InteractiveMarker> marker_;
};
