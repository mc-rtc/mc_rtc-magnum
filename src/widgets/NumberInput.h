#pragma once

#include "details/SingleInput.h"

struct NumberInput : public SingleInput<double>
{
  inline NumberInput(Client & client, const ElementId & id) : SingleInput<double>(client, id) {}

  ~NumberInput() override = default;

  void setupBuffer() override
  {
    buffer_ = data_;
  }

  double dataFromBuffer() override
  {
    return buffer_;
  }

  inline void draw2D() override
  {
    double * data = busy_ ? &buffer_ : &data_;
    SingleInput::draw2D(ImGui::InputDouble, data, 0.0, 0.0, "%.6g");
    if(ImGui::IsItemHovered())
    {
      ImGui::BeginTooltip();
      ImGui::Text("%f", *data);
      ImGui::EndTooltip();
    }
  }

private:
  double buffer_;
};
