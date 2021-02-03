#pragma once

#include "Widget.h"

struct Table : public Widget
{
  Table(Client & client, const ElementId & id) : Widget(client, id) {}

  void start(const std::vector<std::string> & header)
  {
    header_ = header;
    data_.clear();
  }

  void row(const std::vector<std::string> & row)
  {
    data_.push_back(row);
  }

  void end() {}

  void draw2D() override
  {
    auto drawVec = [](const std::vector<std::string> & vec) {
      for(const auto & v : vec)
      {
        ImGui::Text("%s", v.c_str());
        ImGui::NextColumn();
      }
    };
    ImGui::Text("%s", id.name.c_str());
    ImGui::Columns(header_.size());
    drawVec(header_);
    for(const auto & d : data_)
    {
      drawVec(d);
    }
    ImGui::Columns(1);
  }

private:
  std::vector<std::string> header_;
  std::vector<std::vector<std::string>> data_;
};
