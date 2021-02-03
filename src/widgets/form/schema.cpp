#include "schema.h"

namespace
{

template<typename T>
std::optional<T> get_default(const mc_rtc::Configuration & conf)
{
  if(!conf.has("default"))
  {
    return std::nullopt;
  }
  try
  {
    return conf("default");
  }
  catch(mc_rtc::Configuration::Exception & exc)
  {
    exc.silence();
    return std::nullopt;
  }
}

size_t span(size_t minS, size_t maxS)
{
  if(minS == maxS)
  {
    if(minS == 9)
    {
      return 3;
    }
    if(minS == 36)
    {
      return 6;
    }
  }
  return 1;
}

template<typename T>
T default_(size_t id, size_t minS, size_t maxS)
{
  auto span_ = span(minS, maxS);
  if(span_ == 1)
  {
    return 0;
  }
  else if(((id - 1) % span_) == ((id - 1) / span_))
  {
    return 1;
  }
  return 0;
}

} // namespace

namespace form
{

ArrayForm::ArrayForm(const ::Widget & parent, const std::string & name, const mc_rtc::Configuration & schema)
: Widget(parent, name), schema_(schema)
{
  if(!schema_.has("items"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} is an array without items", name);
  }
  auto items = schema("items");
  if(!items.has("type"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} is an array without items' type", name);
  }
  std::string type = items("type");
  isArrayOfObject_ = type == "object";
  isArrayOfArray_ = type == "array";
  minSize_ = schema("minItems", 0);
  maxSize_ = schema("maxItems", std::numeric_limits<unsigned int>::max());
  for(size_t i = 0; i < minSize_; ++i)
  {
    addWidget();
  }
}

bool ArrayForm::ready()
{
  if(isArrayOfObject_)
  {
    return std::all_of(widgets_.begin(), widgets_.end(), [](const auto & w) { return w->ready(); });
  }
  return !isArrayOfArray_ && std::any_of(widgets_.begin(), widgets_.end(), [](const auto & w) { return w->ready(); });
}

void ArrayForm::draw()
{
  if(isArrayOfArray_)
  {
    return;
  }
  if(ImGui::CollapsingHeader(label(name_).c_str()))
  {
    ImGui::Indent();
    size_t removeAt = widgets_.size();
    ImGui::Columns(span(minSize_, maxSize_));
    for(size_t i = 0; i < widgets_.size(); ++i)
    {
      widgets_[i]->draw();
      if(widgets_.size() > minSize_)
      {
        ImGui::SameLine();
        if(ImGui::Button(label("-", i).c_str()))
        {
          removeAt = i;
        }
      }
      ImGui::NextColumn();
    }
    if(widgets_.size() < maxSize_ && ImGui::Button(label("+").c_str()))
    {
      addWidget();
    }
    removeWidget(removeAt);
    ImGui::Unindent();
    ImGui::Columns(1);
  }
}

void ArrayForm::collect(mc_rtc::Configuration & out)
{
  assert(ready());
  auto array = out.array(name(), widgets_.size());
  // FIXME Not very nice
  for(size_t i = 0; i < widgets_.size(); ++i)
  {
    mc_rtc::Configuration temp;
    widgets_[i]->collect(temp);
    array.push(temp(widgets_[i]->name()));
  }
}

void ArrayForm::addWidget()
{
  WidgetPtr widget;
  std::string type = schema_("items")("type");
  std::string nextName = fmt::format("##{}##{}", id_++, fullName());
  if(type == "boolean")
  {
    widget = std::make_unique<Checkbox>(parent_, nextName, std::nullopt, false);
  }
  else if(type == "integer")
  {
    widget = std::make_unique<IntegerInput>(parent_, nextName, std::nullopt, default_<int>(id_, minSize_, maxSize_));
  }
  else if(type == "number")
  {
    widget = std::make_unique<NumberInput>(parent_, nextName, std::nullopt, default_<double>(id_, minSize_, maxSize_));
  }
  else if(type == "string")
  {
    widget = std::make_unique<StringInput>(parent_, nextName);
  }
  else if(type == "object")
  {
    widget = std::make_unique<ObjectForm>(parent_, nextName, schema_("items")("properties"),
                                          schema_("items")("required", std::vector<std::string>{}));
  }
  else if(type == "array")
  {
    return;
  }
  else
  {
    mc_rtc::log::error("Unkown type {} in {}", type, name_);
  }
  if(widget)
  {
    widgets_.push_back(std::move(widget));
  }
}

void ArrayForm::removeWidget(size_t idx)
{
  if(idx >= widgets_.size())
  {
    return;
  }
  widgets_.erase(widgets_.begin() + idx);
}

ObjectForm::ObjectForm(const ::Widget & parent,
                       const std::string & name,
                       const std::map<std::string, mc_rtc::Configuration> & properties,
                       const std::vector<std::string> & required)
: Widget(parent, name)
{
  for(const auto & p : properties)
  {
    if(p.first == "completion")
    {
      continue;
    }
    bool is_required = std::find(required.begin(), required.end(), p.first) != required.end();
    bool is_robot = false;
    std::unique_ptr<form::Widget> widget;
    std::string nextName = fmt::format("{}##{}", p.first, name);
    if(p.second.has("enum"))
    {
      widget = std::make_unique<ComboInput>(parent, nextName, p.second("enum"), false);
    }
    else
    {
      std::string type = p.second("type", std::string(""));
      if(type == "boolean")
      {
        widget = std::make_unique<Checkbox>(parent, nextName, get_default<bool>(p.second));
      }
      else if(type == "integer")
      {
        if(p.first == "robotIndex")
        {
          widget = std::make_unique<DataComboInput>(parent, nextName, std::vector<std::string>{"robots"}, true);
          is_robot = true;
        }
        else
        {
          widget = std::make_unique<IntegerInput>(parent, nextName, get_default<int>(p.second));
        }
      }
      else if(type == "number")
      {
        widget = std::make_unique<NumberInput>(parent, nextName, get_default<double>(p.second));
      }
      else if(type == "string")
      {
        if(p.first == "robot" || p.first == "r1" || p.first == "r2")
        {
          widget = std::make_unique<DataComboInput>(parent, nextName, std::vector<std::string>{"robots"}, false);
          is_robot = true;
        }
        else if(p.first == "body")
        {
          widget = std::make_unique<DataComboInput>(
              parent, nextName, std::vector<std::string>{"bodies", fmt::format("$robot##{}", name)}, false);
        }
        else if(p.first == "surface" || p.first == "r1Surface" || p.first == "r2Surface")
        {
          std::string key = p.first == "surface" ? "$robot" : p.first == "r1Surface" ? "$r1" : "$r2";
          key = fmt::format("{}##{}", key, name);
          widget = std::make_unique<DataComboInput>(parent, nextName, std::vector<std::string>{"surfaces", key}, false);
        }
        else
        {
          widget = std::make_unique<StringInput>(parent, nextName, get_default<std::string>(p.second));
        }
      }
      else if(type == "array")
      {
        widget = std::make_unique<ArrayForm>(parent, nextName, p.second);
      }
      else if(type == "object")
      {
        widget = std::make_unique<ObjectForm>(parent, nextName, p.second("properties"),
                                              p.second("required", std::vector<std::string>{}));
      }
      else
      {
        mc_rtc::log::error("Cannot handle unknown type {} for property {} in {}", type, p.first, name);
      }
    }
    if(!widget)
    {
      mc_rtc::log::error("Failed to load a widget for property {} in {}", p.first, name);
      continue;
    }
    if(is_required)
    {
      required_.push_back(std::move(widget));
    }
    else if(is_robot)
    {
      required_.insert(required_.begin(), std::move(widget));
    }
    else
    {
      widgets_.push_back(std::move(widget));
    }
  }
  std::sort(widgets_.begin(), widgets_.end(), [](const WidgetPtr & lhs, const WidgetPtr & rhs) {
    // lhs is trivial, it's "smaller" if rhs is non trivial or rhs is trivial and has a smaller name
    if(lhs->trivial())
    {
      return !rhs->trivial() || (rhs->trivial() && lhs->fullName() < rhs->fullName());
    }
    // lhs is non-trivial, it's smaller than rhs if rhs is also non-trivial and has a smaller name
    else
    {
      return !rhs->trivial() && lhs->fullName() < rhs->fullName();
    }
  });
}

bool ObjectForm::ready()
{
  return std::all_of(required_.begin(), required_.end(), [](auto && w) { return w->ready(); });
}

void ObjectForm::draw(bool show_header)
{
  if(!show_header || ImGui::CollapsingHeader(label(name_).c_str()))
  {
    if(show_header)
    {
      ImGui::Indent();
    }
    for(auto & w : required_)
    {
      w->draw();
      ImGui::Separator();
    }
    if(widgets_.size() && ImGui::CollapsingHeader(label("Optional fields").c_str()))
    {
      ImGui::Separator();
      ImGui::Indent();
      for(auto & w : widgets_)
      {
        w->draw();
        ImGui::Separator();
      }
      ImGui::Unindent();
    }
    if(show_header)
    {
      ImGui::Unindent();
    }
  }
}

void ObjectForm::collect(mc_rtc::Configuration & out)
{
  assert(ready());
  for(auto & w : required_)
  {
    w->collect(out);
  }
  for(auto & w : widgets_)
  {
    if(w->ready())
    {
      w->collect(out);
    }
  }
}

void ObjectForm::draw()
{
  draw(true);
}

std::optional<std::string> ObjectForm::value(const std::string & name) const
{
  auto value_ = [&name](const std::vector<WidgetPtr> & widgets) -> std::optional<std::string> {
    for(const auto & w : widgets)
    {
      if(w->fullName() == name)
      {
        return w->value();
      }
      else
      {
        auto obj = dynamic_cast<const ObjectForm *>(w.get());
        if(obj)
        {
          auto value = obj->value(name);
          if(value)
          {
            return value;
          }
        }
      }
    }
    return std::nullopt;
  };
  auto req = value_(required_);
  return req ? req : value_(widgets_);
}

} // namespace form
