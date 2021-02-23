#pragma once

#include "details/TransformBase.h"

struct Point3D : public TransformBase<ControlAxis::TRANSLATION>
{
  Point3D(Client & client, const ElementId & id, const ElementId & requestId);

  ~Point3D() override = default;

  void data(bool ro, const Eigen::Vector3d & pos, const mc_rtc::gui::PointConfig & config);

  void draw3D() override;

private:
  mc_rtc::gui::PointConfig config_;
};
