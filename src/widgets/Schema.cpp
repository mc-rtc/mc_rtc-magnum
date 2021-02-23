#include "Schema.h"

#include "form/schema.h"

#include <mc_rtc/config.h>

namespace details
{

inline bfs::path current_path()
{
  auto cwd = get_current_dir_name();
  bfs::path out(cwd);
  free(cwd);
  return out;
}

inline bfs::path canonical(const bfs::path & p)
{
#ifndef __EMSCRIPTEN__
  return bfs::canonical(p);
#else
  return bfs::canonical(p, current_path());
#endif
}

} // namespace details

namespace
{

std::string removeFakeDir(const std::string & in)
{
  std::string_view fakeDir = "/../";
  if(in.size() >= fakeDir.size() && in.substr(0, fakeDir.size()) == fakeDir)
  {
    return in.substr(fakeDir.size());
  }
  if(in.size() >= fakeDir.size() && in.substr(0, fakeDir.size() - 1) == fakeDir.substr(1))
  {
    return in.substr(fakeDir.size() - 1);
  }
  return in;
}

void resolveRef(const bfs::path & path,
                mc_rtc::Configuration conf,
                const std::function<mc_rtc::Configuration(const bfs::path &)> & loadFn)
{
  if(conf.size())
  {
    for(size_t i = 0; i < conf.size(); ++i)
    {
      resolveRef(path, conf[i], loadFn);
    }
  }
  else
  {
    auto keys = conf.keys();
    for(const auto & k : keys)
    {
      if(k == "$ref")
      {
        auto ref = loadFn(details::canonical(path.parent_path() / removeFakeDir(conf(k)).c_str()));
        auto refKeys = ref.keys();
        for(const auto & rk : refKeys)
        {
          if(!conf.has(rk))
          {
            conf.add(rk, ref(rk));
          }
        }
        conf.remove("$ref");
      }
      else
      {
        resolveRef(path, conf(k), loadFn);
      }
    }
  }
}

void resolveAllOf(mc_rtc::Configuration conf)
{
  if(conf.size())
  {
    for(size_t i = 0; i < conf.size(); ++i)
    {
      resolveAllOf(conf[i]);
    }
  }
  else
  {
    auto keys = conf.keys();
    for(const auto & k : keys)
    {
      if(k == "allOf")
      {
        std::vector<mc_rtc::Configuration> allOf = conf("allOf");
        for(auto & c : allOf)
        {
          resolveAllOf(c);
          conf.load(c);
        }
        conf.remove("allOf");
      }
      else
      {
        resolveAllOf(conf(k));
      }
    }
  }
}

} // namespace

struct SchemaForm
{
  SchemaForm(const ::Widget & parent, const std::string & name, const mc_rtc::Configuration & schema)
  {
    if(!schema.has("properties"))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("{} is not a correct schema, it has no properties");
    }
    object_ = std::make_unique<form::ObjectForm>(parent, name, schema("properties"),
                                                 schema("required", std::vector<std::string>{}));
  }

  bool draw(const char * label)
  {
    ImGui::Indent();
    object_->draw(false);
    ImGui::Unindent();
    return ImGui::Button(label);
  }

  mc_rtc::Configuration data()
  {
    mc_rtc::Configuration out;
    object_->collect(out);
    return out;
  }

  const std::string & title()
  {
    return object_->fullName();
  }

  std::optional<std::string> value(const std::string & name) const
  {
    return object_->value(name);
  }

  bool ready() const
  {
    return object_->ready();
  }

private:
  std::unique_ptr<form::ObjectForm> object_;
};

Schema::Schema(Client & client, const ElementId & id) : Widget(client, id) {}

Schema::~Schema() {}

void Schema::data(const std::string & schema)
{
  if(schema == schema_)
  {
    return;
  }
  form_.reset(nullptr);
  schema_ = schema;
#ifndef __EMSCRIPTEN__
  bfs::path all_schemas = bfs::path(mc_rtc::INSTALL_PREFIX) / "share" / "doc" / "mc_rtc" / "json" / "schemas";
#else
  bfs::path all_schemas = bfs::path("/assets/schemas");
#endif
  bfs::path schema_dir = all_schemas / schema_.c_str();
  if(!bfs::exists(schema_dir) || !bfs::is_directory(schema_dir))
  {
    mc_rtc::log::error("Cannot load schema from non existing directory: {}", schema_dir.string());
    return;
  }
  bfs::directory_iterator dit(schema_dir), endit;
  std::vector<bfs::path> schemas;
  std::copy(dit, endit, std::back_inserter(schemas));
  for(const auto & s : schemas)
  {
    auto & schema = loadSchema(s);
    schemas_[schema("title")] = schema;
  }
}

void Schema::draw2D()
{
  const char * label_ = form_ ? form_->title().c_str() : "";
  if(ImGui::BeginCombo(label("", "schemaSelector").c_str(), label_))
  {
    for(auto & s : schemas_)
    {
      bool selected = form_ && form_->title() == s.first;
      if(ImGui::Selectable(s.first.c_str(), selected))
      {
        form_ = std::make_unique<SchemaForm>(*this, s.first, s.second);
      }
      if(selected)
      {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
  if(form_)
  {
    if(form_->draw(id.name.c_str()) && form_->ready())
    {
      auto data = form_->data();
      client.send_request(id, data);
      form_ = std::make_unique<SchemaForm>(*this, form_->title(), schemas_[form_->title()]);
    }
  }
}

std::optional<std::string> Schema::value(const std::string & name) const
{
  if(form_)
  {
    return form_->value(name);
  }
  return "";
}

mc_rtc::Configuration & Schema::loadSchema(const bfs::path & path)
{
  if(details::canonical(path) != path)
  {
    return loadSchema(details::canonical(path));
  }
  if(all_schemas_.count(path.string()))
  {
    return all_schemas_[path.string()];
  }
  if(!bfs::exists(path))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No schema can be loaded from {}", path.string());
  }
  auto & schema = all_schemas_[path.string()];
  schema.load(path.string());
  resolveRef(path, schema, [this](const bfs::path & p) { return loadSchema(p); });
  resolveAllOf(schema);
  return schema;
}
