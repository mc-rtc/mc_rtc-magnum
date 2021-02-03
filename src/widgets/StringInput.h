#pragma once

#include "details/SingleInput.h"

struct StringInput : public SingleInput<std::string>
{
  inline StringInput(Client & client, const ElementId & id) : SingleInput<std::string>(client, id) {}

  ~StringInput() override = default;

  void setupBuffer() override
  {
    buffer_.resize(std::max<size_t>(256, data_.size() + 1));
    std::memcpy(&buffer_[0], data_.c_str(), data_.size() + 1);
  }

  std::string dataFromBuffer() override
  {
    return {buffer_.data(), strnlen(buffer_.data(), buffer_.size())};
  }

  inline void draw2D() override
  {
    char * data = busy_ ? buffer_.data() : data_.data();
    size_t data_len = busy_ ? buffer_.size() : data_.size();
    auto InputText = [](const char * label, char * buffer, size_t len, int flags) {
      return ImGui::InputText(label, buffer, len, flags);
    };
    SingleInput::draw2D(InputText, data, data_len);
  }

private:
  std::vector<char> buffer_;
};
