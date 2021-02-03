#pragma once

#include "Widget.h"

struct NumberSlider : public Widget
{
  inline NumberSlider(Client & client, const ElementId & id) : Widget(client, id) {}

  ~NumberSlider() override = default;

  inline void data(double data, double min, double max)
  {
    data_ = data;
    min_ = min;
    max_ = max;
  }

  inline void draw2D() override
  {
    if(ImGui::SliderFloat(id.name.c_str(), &data_, min_, max_))
    {
      client.send_request(id, data_);
    }
  }

private:
  float data_;
  float min_;
  float max_;
};
