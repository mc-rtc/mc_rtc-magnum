#pragma once

#include "details/TransformBase.h"

namespace mc_rtc::magnum
{

struct Rotation : public TransformBase<ControlAxis::ROTATION>
{
  Rotation(Client & client, const ElementId & id, const ElementId & reqId) : TransformBase(client, id, reqId) {}

  void draw3D() override
  {
    TransformBase::draw3D();
    client.gui().drawFrame(convert(marker_.pose()));
  }
};

} // namespace mc_rtc::magnum
