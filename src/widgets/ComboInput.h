#pragma once

#include "Widget.h"

struct ComboInput : public Widget
{
  inline ComboInput(Client & client, const ElementId & id) : Widget(client, id) {}

  ~ComboInput() override = default;

  inline void data(const std::vector<std::string> & values, const std::string & data)
  {
    values_ = values;
    data_ = data;
  }

  inline void draw2D() override
  {
    size_t idx = [&]() {
      size_t i = 0;
      for(; i < values_.size(); ++i)
      {
        if(values_[i] == data_)
        {
          return i;
        }
      }
      return i;
    }();
    const char * label_ = idx < values_.size() ? data_.c_str() : "";
    if(ImGui::BeginCombo(label(id.name).c_str(), label_))
    {
      for(size_t i = 0; i < values_.size(); ++i)
      {
        if(ImGui::Selectable(values_[i].c_str(), idx == i))
        {
          if(i != idx)
          {
            data_ = values_[i];
            client.send_request(id, data_);
          }
        }
        if(idx == i)
        {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
  }

protected:
  std::vector<std::string> values_;
  std::string data_;
};
