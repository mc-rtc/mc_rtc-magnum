#include "widgets.h"

#include "../Form.h"
#include "../Schema.h"

namespace form
{

ArrayInput::ArrayInput(const ::Widget & parent,
                       const std::string & name,
                       const Eigen::VectorXd & default_,
                       bool fixed_size)
: SimpleInput::SimpleInput(parent, name, default_), fixed_(fixed_size)
{
}

void ArrayInput::draw()
{
  ImGui::Columns(2);
  for(size_t i = 0; i < static_cast<size_t>(temp_.size()); ++i)
  {
    if(ImGui::InputDouble(label(fmt::format("{}", i)).c_str(), &temp_(i)))
    {
      value_ = temp_;
    }
    if(!fixed_)
    {
      ImGui::SameLine();
      if(ImGui::Button(label("-", i).c_str()))
      {
        Eigen::VectorXd nValue = Eigen::VectorXd::Zero(temp_.size() - 1);
        if(i != 0)
        {
          nValue.head(i) = temp_.head(i);
        }
        if(nValue.size() - i)
        {
          nValue.tail(nValue.size() - i) = temp_.tail(temp_.size() - 1 - i);
        }
        temp_ = nValue;
        value_ = temp_;
      }
    }
  }
  if(!fixed_)
  {
    if(ImGui::Button(label("+").c_str()))
    {
      Eigen::VectorXd nValue = Eigen::VectorXd::Zero(temp_.size() + 1);
      nValue.head(temp_.size()) = temp_;
      temp_ = nValue;
      value_ = temp_;
    }
  }
  ImGui::NextColumn();
  ImGui::Text("%s", name().c_str());
  ImGui::Columns(1);
}

ComboInput::ComboInput(const ::Widget & parent,
                       const std::string & name,
                       const std::vector<std::string> & values,
                       bool send_index)
: SimpleInput(parent, name, ""), values_(values), idx_(values_.size()), send_index_(send_index)
{
  if(values_.size() == 1)
  {
    value_ = values_[0];
    idx_ = 0;
  }
}

void ComboInput::draw()
{
  const char * label = idx_ < values_.size() ? values_[idx_].c_str() : "";
  draw(label);
}

void ComboInput::draw(const char * label_)
{
  if(values_.size() == 1 && value_.has_value())
  {
    return;
  }
  if(ImGui::BeginCombo(label(name_).c_str(), label_))
  {
    for(size_t i = 0; i < values_.size(); ++i)
    {
      if(ImGui::Selectable(values_[i].c_str(), idx_ == i))
      {
        idx_ = i;
        value_ = values_[i];
      }
      if(idx_ == i)
      {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndCombo();
  }
}

DataComboInput::DataComboInput(const ::Widget & parent,
                               const std::string & name,
                               const std::vector<std::string> & ref,
                               bool send_index)
: ComboInput(parent, name, {}, send_index), ref_(ref)
{
}

void DataComboInput::draw()
{
  auto getValue = [&](const std::string & value) {
    auto * form_ptr = dynamic_cast<const Form *>(&parent_);
    if(form_ptr)
    {
      return form_ptr->value(value);
    }
    auto * schema_ptr = dynamic_cast<const Schema *>(&parent_);
    if(schema_ptr)
    {
      return schema_ptr->value(value).value_or("");
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("Form element outisde of Form or Schema");
  };
  auto data = parent_.client.data();
  std::string label;
  auto resolve = [&]() -> std::vector<std::string> {
    for(size_t i = 0; i < ref_.size(); ++i)
    {
      std::string ref = ref_[i];
      if(ref.size() && ref[0] == '$')
      {
        ref = getValue(ref.substr(1));
      }
      if(!data.has(ref))
      {
        if(ref_[i].size() && ref_[i][0] == '$')
        {
          label = fmt::format("Fill {} first", ref_[i].substr(1));
        }
        else
        {
          std::string full_ref = "";
          for(size_t j = 0; j <= i; ++j)
          {
            full_ref += ref_[j];
            if(j != i)
            {
              full_ref += "/";
            }
          }
          label = fmt::format("No {} entry in the data provided by the server", full_ref);
        }
        return {};
      }
      data = data(ref);
    }
    return data;
  };
  values_ = resolve();
  if(idx_ >= values_.size() || values_[idx_] != value_)
  {
    idx_ = values_.size();
    value_ = std::nullopt;
  }
  const char * label_ = label.size() ? label.c_str() : value_.has_value() ? value_.value().c_str() : "";
  ComboInput::draw(label_);
}

} // namespace form
