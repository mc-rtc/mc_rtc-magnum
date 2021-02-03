#pragma once

#include "details/SingleInput.h"

struct IntegerInput : public SingleInput<int>
{
  inline IntegerInput(Client & client, const ElementId & id) : SingleInput<int>(client, id) {}

  ~IntegerInput() override = default;

  void setupBuffer() override
  {
    buffer_ = data_;
  }

  int dataFromBuffer() override
  {
    return buffer_;
  }

  inline void draw2D() override
  {
    int * data = busy_ ? &buffer_ : &data_;
    SingleInput::draw2D(ImGui::InputInt, data, 0, 0);
  }

private:
  int buffer_;
};
