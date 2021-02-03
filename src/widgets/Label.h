#pragma once

#include "Widget.h"

struct Label : public Widget
{
  inline Label(Client & client, const ElementId & id) : Widget(client, id) {}

  ~Label() override = default;

  inline void data(const std::string & txt)
  {
    txt_ = txt;
  }

  inline void draw2D() override
  {
    ImGui::LabelText(txt_.c_str(), "%s", id.name.c_str());
  }

private:
  std::string txt_;
};
