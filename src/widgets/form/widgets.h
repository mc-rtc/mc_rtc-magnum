#pragma once

#include "../Widget.h"

#include <memory>
#include <optional>

namespace form
{

struct Widget
{
  Widget(const ::Widget & parent, const std::string & name) : parent_(parent), name_(name) {}

  virtual ~Widget() = default;

  virtual bool ready() = 0;

  virtual void draw() = 0;

  /** A form widget is trivial if it doesn't contain other widgets */
  inline virtual bool trivial() const
  {
    return true;
  }

  inline virtual std::string value()
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("::value() is not implemented for this form element");
  }

  virtual void collect(mc_rtc::Configuration & out) = 0;

  template<typename T = const char *>
  inline std::string label(std::string_view label, T && suffix = "")
  {
    return fmt::format("{}##{}{}{}{}", label, parent_.id.category, parent_.id.name, name_, suffix);
  }

  inline std::string name() const
  {
    size_t pos = name_.find("##");
    if(pos != std::string::npos)
    {
      return name_.substr(0, pos);
    }
    return name_;
  }

  inline const std::string & fullName() const
  {
    return name_;
  }

  bool required;

  inline const ::Widget & parent() const
  {
    return parent_;
  }

protected:
  const ::Widget & parent_;
  std::string name_;
};

template<typename DataT>
struct SimpleInput : public Widget
{
  SimpleInput(const ::Widget & parent, const std::string & name) : Widget(parent, name) {}

  SimpleInput(const ::Widget & parent,
              const std::string & name,
              const std::optional<DataT> & value,
              const std::optional<DataT> & temp = std::nullopt)
  : Widget(parent, name), value_(value)
  {
    if(temp.has_value())
    {
      temp_ = temp.value();
    }
    else if(value_.has_value())
    {
      temp_ = value.value();
    }
    else
    {
      if constexpr(std::is_arithmetic_v<DataT>)
      {
        temp_ = 0;
      }
      else
      {
        temp_ = {};
      }
    }
  }

  ~SimpleInput() override = default;

  bool ready() override
  {
    if constexpr(std::is_same_v<DataT, std::string> || std::is_same_v<DataT, Eigen::VectorXd>)
    {
      return value_.has_value() && value_.value().size() != 0;
    }
    else
    {
      return value_.has_value();
    }
  }

  void collect(mc_rtc::Configuration & out) override
  {
    if(ready())
    {
      out.add(name(), value_.value());
    }
    else
    {
      out.add(name(), temp_);
    }
  }

  std::string value() override
  {
    if constexpr(std::is_same_v<DataT, std::string>)
    {
      return value_.has_value() ? value_.value() : "";
    }
    else
    {
      return fmt::format("{}", value_.has_value() ? value_.value() : temp_);
    }
  }

protected:
  std::optional<DataT> value_;
  DataT temp_;
};

struct Checkbox : public SimpleInput<bool>
{
  using SimpleInput::SimpleInput;

  inline void draw() override
  {
    if(ImGui::Checkbox(label(name_).c_str(), &temp_))
    {
      value_ = temp_;
    }
  }
};

struct IntegerInput : public SimpleInput<int>
{
  using SimpleInput::SimpleInput;

  inline void draw() override
  {
    if(ImGui::InputInt(label(name_).c_str(), &temp_, 0, 0))
    {
      value_ = temp_;
    }
  }
};

struct NumberInput : public SimpleInput<double>
{
  using SimpleInput::SimpleInput;

  inline void draw() override
  {
    if(ImGui::InputDouble(label(name_).c_str(), &temp_))
    {
      value_ = temp_;
    }
  }
};

struct StringInput : public SimpleInput<std::string>
{
  using SimpleInput::SimpleInput;

  inline void draw() override
  {
    const auto & value = value_.has_value() ? value_.value() : "";
    if(buffer_.size() < std::max<size_t>(value.size() + 1, 256))
    {
      buffer_.resize(std::max<size_t>(value.size() + 1, 256));
      std::memcpy(buffer_.data(), value.data(), value.size());
      buffer_[value.size()] = 0;
    }
    if(ImGui::InputText(label(name_).c_str(), buffer_.data(), buffer_.size()))
    {
      value_ = {buffer_.data(), strnlen(buffer_.data(), buffer_.size())};
    }
  }

private:
  std::vector<char> buffer_;
};

struct ArrayInput : public SimpleInput<Eigen::VectorXd>
{
  ArrayInput(const ::Widget & parent, const std::string & name, const Eigen::VectorXd & default_, bool fixed_size);

  void draw() override;

private:
  bool fixed_;
};

struct ComboInput : public SimpleInput<std::string>
{
  ComboInput(const ::Widget & parent,
             const std::string & name,
             const std::vector<std::string> & values,
             bool send_index);

  void draw() override;

  inline void collect(mc_rtc::Configuration & out) override
  {
    assert(ready());
    if(send_index_)
    {
      out.add(name(), static_cast<unsigned int>(idx_));
    }
    else
    {
      out.add(name(), value_.value());
    }
  }

protected:
  std::vector<std::string> values_;
  size_t idx_;
  bool send_index_;

  void draw(const char * label);
};

struct DataComboInput : public ComboInput
{
  DataComboInput(const ::Widget & parent,
                 const std::string & name,
                 const std::vector<std::string> & ref,
                 bool send_index);

  void draw() override;

protected:
  std::vector<std::string> ref_;
};

using WidgetPtr = std::unique_ptr<Widget>;

} // namespace form
