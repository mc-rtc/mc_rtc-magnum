#pragma once

#include "Widget.h"

struct Checkbox : public Widget
{
  inline Checkbox(Client & client, const ElementId & id) : Widget(client, id) {}

  ~Checkbox() override = default;

  inline void data(bool data)
  {
    data_ = data;
  }

  inline void draw2D() override
  {
    if(ImGui::Checkbox(label(id.name).c_str(), &data_))
    {
      client.send_request(id);
    }
  }

private:
  bool data_ = false;
};
