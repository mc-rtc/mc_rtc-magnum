#pragma once

#include <memory>

#include <SpaceVecAlg/SpaceVecAlg>

#include "../../Camera.h"
#include "ControlAxis.h"

struct InteractiveMarker
{
  InteractiveMarker(const sva::PTransformd & pose = sva::PTransformd::Identity(), ControlAxis mask = ControlAxis::NONE);

  void mask(ControlAxis mask);

  void pose(const sva::PTransformd & pose);

  inline const sva::PTransformd & pose() const noexcept
  {
    return pose_;
  }

  bool draw(const Camera & camera);

private:
  sva::PTransformd pose_;
  int operation_;
  bool active_ = false;
  int id_;
  static int next_id_;
};

using InteractiveMarkerPtr = std::unique_ptr<InteractiveMarker>;
