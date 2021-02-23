#include "Category.h"

#include "imgui.h"

void Category::draw2D()
{
  int sid = -100;
  for(auto & w : widgets)
  {
    if(w->id.sid == sid && sid != -1)
    {
      ImGui::SameLine();
    }
    else if(sid != -100)
    {
      ImGui::Separator();
    }
    w->draw2D();
    sid = w->id.sid;
  }
  if(categories.size())
  {
    if(widgets.size())
    {
      ImGui::Separator();
    }
    ImGui::Indent();
    std::sort(categories.begin(), categories.end(),
              [](const auto & lhs, const auto & rhs) { return lhs->name < rhs->name; });
    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_Reorderable;
    if(ImGui::BeginTabBar(name.c_str(), tab_bar_flags))
    {
      for(auto & cat : categories)
      {
        if(ImGui::BeginTabItem(cat->name.c_str()))
        {
          cat->draw2D();
          ImGui::EndTabItem();
        }
      }
      ImGui::EndTabBar();
    }
    ImGui::Unindent();
  }
}

void Category::draw3D()
{
  for(auto & w : widgets)
  {
    w->draw3D();
  }
  for(auto & cat : categories)
  {
    cat->draw3D();
  }
}

void Category::started()
{
  for(auto & w : widgets)
  {
    w->seen = false;
  }
  for(auto & cat : categories)
  {
    cat->started();
  }
}

void Category::stopped()
{
  /** Clean up categories first */
  for(auto & cat : categories)
  {
    cat->stopped();
  }
  /** Remove empty categories */
  {
    auto it = std::remove_if(categories.begin(), categories.end(),
                             [](const auto & c) { return c->widgets.size() == 0 && c->categories.size() == 0; });
    categories.erase(it, categories.end());
  }
  /** Remove widgets that have not been seen */
  {
    auto it = std::remove_if(widgets.begin(), widgets.end(), [](const auto & w) { return !w->seen; });
    widgets.erase(it, widgets.end());
  }
}
