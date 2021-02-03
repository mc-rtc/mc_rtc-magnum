#pragma once

#include <mc_control/ControllerClient.h>

#include "Category.h"

struct McRtcGui;

struct Client : public mc_control::ControllerClient
{
  Client(McRtcGui & gui) : mc_control::ControllerClient{}, gui_(gui) {}

  /** Update the client data from the latest server message */
  void update();

  /** Draw ImGui elements */
  void draw2D();

  /** Draw 3D elements */
  void draw3D(Camera & camera);

  /** Remove all elements */
  void clear();

  inline const mc_rtc::Configuration & data() const noexcept
  {
    return data_;
  }

  McRtcGui & gui() noexcept
  {
    return gui_;
  }

private:
  McRtcGui & gui_;

  std::vector<char> buffer_ = std::vector<char>(65535);
  std::chrono::system_clock::time_point t_last_ = std::chrono::system_clock::now();

  /** No message for unsupported types */
  void default_impl(const std::string &, const ElementId &) final {}

  void started() override;

  void category(const std::vector<std::string> &, const std::string &) override;

  void label(const ElementId & id, const std::string & txt) override;

  void array_label(const ElementId & id,
                   const std::vector<std::string> & labels,
                   const Eigen::VectorXd & data) override;

  void button(const ElementId & id) override;

  void checkbox(const ElementId & id, bool state) override;

  void string_input(const ElementId & id, const std::string & data) override;

  void integer_input(const ElementId & id, int data) override;

  void number_input(const ElementId & id, double data) override;

  void number_slider(const ElementId & id, double data, double min, double max) override;

  void array_input(const ElementId & id,
                   const std::vector<std::string> & labels,
                   const Eigen::VectorXd & data) override;

  void combo_input(const ElementId & id, const std::vector<std::string> & values, const std::string & data) override;

  void data_combo_input(const ElementId & id, const std::vector<std::string> & ref, const std::string & data) override;

  // void point3d(const ElementId & id,
  //             const ElementId & requestId,
  //             bool ro,
  //             const Eigen::Vector3d & pos,
  //             const mc_rtc::gui::PointConfig & config) override;

  void trajectory(const ElementId & id,
                  const std::vector<Eigen::Vector3d> & points,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const ElementId & id,
                  const std::vector<sva::PTransformd> & points,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const ElementId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const ElementId & id,
                  const sva::PTransformd & point,
                  const mc_rtc::gui::LineConfig & config) override;

  // void polygon(const ElementId & id,
  //             const std::vector<std::vector<Eigen::Vector3d>> & points,
  //             const mc_rtc::gui::LineConfig & config) override;

  // inline void polygon(const ElementId & id,
  //             const std::vector<std::vector<Eigen::Vector3d>> & points,
  //             const mc_rtc::gui::Color & color) override
  //{
  //  polygon(id, points, mc_rtc::gui::LineConfig(color));
  //}

  // void force(const ElementId & id,
  //           const ElementId & requestId,
  //           const sva::ForceVecd & force,
  //           const sva::PTransformd & pos,
  //           const mc_rtc::gui::ForceConfig & forceConfig,
  //           bool /* ro */) override;

  // void arrow(const ElementId & id,
  //           const ElementId & requestId,
  //           const Eigen::Vector3d & start,
  //           const Eigen::Vector3d & end,
  //           const mc_rtc::gui::ArrowConfig & config,
  //           bool ro) override;

  // void rotation(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos) override;

  // void transform(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos) override;

  // void xytheta(const ElementId & id,
  //             const ElementId & requestId,
  //             bool ro,
  //             const Eigen::Vector3d & xytheta,
  //             double altitude) override;

  void table_start(const ElementId & id, const std::vector<std::string> & header) override;

  void table_row(const ElementId & id, const std::vector<std::string> & data) override;

  void table_end(const ElementId & id) override;

  // void robot(const ElementId & id,
  //           const std::vector<std::string> & params,
  //           const std::vector<std::vector<double>> & q,
  //           const sva::PTransformd & posW) override;

  void schema(const ElementId & id, const std::string & schema) override;

  void form(const ElementId & id) override;

  void form_checkbox(const ElementId & formId, const std::string & name, bool required, bool default_) override;

  void form_integer_input(const ElementId & formId, const std::string & name, bool required, int default_) override;

  void form_number_input(const ElementId & formId, const std::string & name, bool required, double default_) override;

  void form_string_input(const ElementId & formId,
                         const std::string & name,
                         bool required,
                         const std::string & default_) override;

  void form_array_input(const ElementId & formId,
                        const std::string & name,
                        bool required,
                        const Eigen::VectorXd & default_,
                        bool fixed_size) override;

  void form_combo_input(const ElementId & formId,
                        const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index) override;

  void form_data_combo_input(const ElementId & formId,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & ref,
                             bool send_index) override;

  void stopped() override;

  Category root_;

  /** Returns a category (creates it if it does not exist */
  Category & getCategory(const std::vector<std::string> & category);

  /** Get a widget with the right type and id */
  template<typename T, typename... Args>
  T & widget(const ElementId & id, Args &&... args)
  {
    auto & category = getCategory(id.category);
    auto it =
        std::find_if(category.widgets.begin(), category.widgets.end(), [&](auto & w) { return w->id.name == id.name; });
    if(it == category.widgets.end())
    {
      auto & w = category.widgets.emplace_back(std::make_unique<T>(*this, id, std::forward<Args>(args)...));
      w->seen = true;
      return *dynamic_cast<T *>(w.get());
    }
    else
    {
      auto w_ptr = dynamic_cast<T *>(it->get());
      if(w_ptr)
      {
        w_ptr->seen = true;
        return *w_ptr;
      }
      /** Different type, remove and add the widget again */
      category.widgets.erase(it);
      return widget<T>(id, std::forward<Args>(args)...);
    }
  }
};
