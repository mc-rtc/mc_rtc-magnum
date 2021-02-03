#pragma once

#include "Widget.h"

#include "form/widgets.h"

// FIXME Does not update if the form is changed between two calls
struct Form : public Widget
{
  Form(Client & client, const ElementId & id) : Widget(client, id) {}

  template<typename WidgetT, typename... Args>
  void widget(const std::string & name, bool required, Args &&... args)
  {
    if(required)
    {
      widget<WidgetT>(name, requiredWidgets_, std::forward<Args>(args)...);
    }
    else
    {
      widget<WidgetT>(name, otherWidgets_, std::forward<Args>(args)...);
    }
  }

  inline std::string value(const std::string & name) const
  {
    auto pred = [&](auto && w) { return w->fullName() == name; };
    auto it = std::find_if(requiredWidgets_.begin(), requiredWidgets_.end(), pred);
    if(it == requiredWidgets_.end())
    {
      it = std::find_if(otherWidgets_.begin(), otherWidgets_.end(), pred);
      if(it == otherWidgets_.end())
      {
        return "";
      }
    }
    return (*it)->value();
  }

  void draw2D()
  {
    for(auto & w : requiredWidgets_)
    {
      w->draw();
    }
    // FIXME Maybe always show if there is few optional elements?
    if(requiredWidgets_.size() == 0 || (otherWidgets_.size() && ImGui::CollapsingHeader(label("Optional").c_str())))
    {
      ImGui::Indent();
      for(auto & w : otherWidgets_)
      {
        w->draw();
      }
      ImGui::Unindent();
    }
    if(ImGui::Button(label(id.name).c_str()))
    {
      for(auto & w : requiredWidgets_)
      {
        if(!w->ready())
        {
          // FIXME SHOW A POPUP WITH ALL REQUESTED FIELDS MISSING
          mc_rtc::log::critical("Form not ready");
          return;
        }
      }
      mc_rtc::Configuration data;
      for(auto & w : requiredWidgets_)
      {
        w->collect(data);
      }
      for(auto & w : otherWidgets_)
      {
        if(w->ready())
        {
          w->collect(data);
        }
      }
      client.send_request(id, data);
    }
  }

protected:
  std::vector<form::WidgetPtr> requiredWidgets_;
  std::vector<form::WidgetPtr> otherWidgets_;

  template<typename WidgetT, typename... Args>
  void widget(const std::string & name, std::vector<form::WidgetPtr> & widgets, Args &&... args)
  {
    auto it = std::find_if(widgets.begin(), widgets.end(), [&](const auto & w) { return w->fullName() == name; });
    if(it == widgets.end())
    {
      widgets.push_back(std::make_unique<WidgetT>(*this, name, std::forward<Args>(args)...));
    }
  }
};
