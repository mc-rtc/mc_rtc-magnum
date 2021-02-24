#pragma once

#include "Widget.h"

namespace details
{

struct RobotImpl;
}

struct Robot : public Widget
{
  Robot(Client & client, const ElementId & id);

  ~Robot() override;

  void data(const std::vector<std::string> & params,
            const std::vector<std::vector<double>> & q,
            const sva::PTransformd & posW);

  void draw2D() override;

  void draw3D() override;

private:
  std::unique_ptr<details::RobotImpl> impl_;
};
