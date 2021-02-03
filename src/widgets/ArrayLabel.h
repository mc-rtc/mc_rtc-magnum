#pragma once

#include "Widget.h"

struct ArrayLabel : public Widget
{
  inline ArrayLabel(Client & client, const ElementId & id) : Widget(client, id) {}

  ~ArrayLabel() override = default;

  inline void data(const std::vector<std::string> & labels, const Eigen::VectorXd & data)
  {
    labels_ = labels;
    data_ = data;
  }

  void draw2D() override
  {
    if(labels_.size())
    {
      ImGui::Columns(3);
      ImGui::Text("%s", id.name.c_str());
      ImGui::NextColumn();
      for(size_t i = 0; i < labels_.size(); ++i)
      {
        ImGui::Text("%s", labels_[i].c_str());
      }
      ImGui::NextColumn();
      ImVec2 min;
      ImVec2 max;
      for(int i = 0; i < data_.size(); ++i)
      {
        ImGui::Text("%.4f", data_(i));
        if(i == 0)
        {
          min = ImGui::GetItemRectMin();
        }
        if(i == data_.size() - 1)
        {
          max = ImGui::GetItemRectMax();
        }
      }
      if(ImGui::IsMouseHoveringRect(min, max))
      {
        ImGui::BeginTooltip();
        ImGui::Text("%s", fmt::format("{:0.4f}", data_.norm()).c_str());
        ImGui::EndTooltip();
      }
      ImGui::NextColumn();
      ImGui::Columns(1);
    }
    else
    {
      if(data_.size() > 6)
      {
        ImGui::LabelText(fmt::format("{:0.4f}", data_.norm()).c_str(), "%s", id.name.c_str());
        if(ImGui::IsItemHovered())
        {
          ImGui::BeginTooltip();
          ImGui::Text("%s", fmt::format("{}", data_).c_str());
          ImGui::EndTooltip();
        }
      }
      else
      {
        ImGui::LabelText(fmt::format("{}", data_).c_str(), "%s", id.name.c_str());
        if(ImGui::IsItemHovered())
        {
          ImGui::BeginTooltip();
          ImGui::Text("%s", fmt::format("{:0.4f}", data_.norm()).c_str());
          ImGui::EndTooltip();
        }
      }
    }
  }

private:
  std::vector<std::string> labels_;
  Eigen::VectorXd data_;
};
