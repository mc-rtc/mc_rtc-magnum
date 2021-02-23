#pragma once

#include "Widget.h"

struct Category;
using CategoryPtr = std::unique_ptr<Category>;

/** Category, the root has depth -1 */
struct Category
{
  Category() = default;
  inline Category(const std::string & name, int depth) : name(name), depth(depth) {}

  std::string name = "";
  int depth = -1;
  std::vector<WidgetPtr> widgets;
  std::vector<CategoryPtr> categories;

  inline bool empty() const
  {
    return widgets.size() == 0 && categories.size() == 0;
  }

  void draw2D();
  void draw3D();
  void started();
  void stopped();
};
