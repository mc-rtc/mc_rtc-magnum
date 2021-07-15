#pragma once

#include "Widget.h"

#include "details/InteractiveMarker.h"

namespace mc_rtc::magnum
{

struct Arrow : public Widget
{
  Arrow(Client & client, const ElementId & id, McRtcGui & gui, const ElementId & reqId)
  : Widget(client, id, gui), requestId_(reqId)
  {
  }

  void data(const Eigen::Vector3d & start,
            const Eigen::Vector3d & end,
            const mc_rtc::gui::ArrowConfig & config,
            bool ro)
  {
    startMarker_.mask(ro ? ControlAxis::NONE : ControlAxis::TRANSLATION);
    startMarker_.pose(start);
    endMarker_.mask(ro ? ControlAxis::NONE : ControlAxis::TRANSLATION);
    endMarker_.pose(end);
    config_ = config;
  }

  void draw3D() override
  {
    const auto & start = startMarker_.pose().translation();
    const auto & end = endMarker_.pose().translation();
    gui_.drawArrow(translation(start), translation(end), config_.shaft_diam, config_.head_diam, config_.head_len,
                   convert(config_.color));
    bool changed = startMarker_.draw(gui_.camera());
    if(endMarker_.draw(gui_.camera()) || changed)
    {
      Eigen::Vector6d data;
      data << start, end;
      client.send_request(requestId_, data);
    }
  }

private:
  ElementId requestId_;
  mc_rtc::gui::ArrowConfig config_;
  InteractiveMarker startMarker_;
  InteractiveMarker endMarker_;
};

} // namespace mc_rtc::magnum
