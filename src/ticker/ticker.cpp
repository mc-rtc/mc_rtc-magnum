#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <chrono>
#include <iostream>

#ifndef MC_RTC_VERSION_MAJOR
static constexpr int MC_RTC_VERSION_MAJOR = mc_rtc::MC_RTC_VERSION[0] - '0';
#endif

template<typename T>
mc_rtc::gui::StateBuilder * get_gui(T & gc)
{
  static_assert(std::is_same_v<T, mc_control::MCGlobalController>);
  if constexpr(MC_RTC_VERSION_MAJOR >= 2)
  {
    return &gc.controller().gui();
  }
  else
  {
    return gc.controller().gui().get();
  }
}

int main(int argc, char * argv[])
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error(
        "mc-rtc-magnum-ticker was compiled with {} but mc_rtc is at version {}, you might face subtle issues "
        "or unexpected crashes, please recompile mc-rtc-magnum-ticker",
        mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }
  std::string conf = argc > 1 ? argv[1] : "";
  mc_control::MCGlobalController controller{conf};

  std::vector<double> q;
  {
    auto & mbc = controller.robot().mbc();
    const auto & rjo = controller.ref_joint_order();
    for(const auto & jn : rjo)
    {
      if(controller.robot().hasJoint(jn))
      {
        for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)])
        {
          q.push_back(qj);
        }
      }
      else
      {
        // FIXME This assumes that a joint that is in ref_joint_order but missing from the robot is of size 1 (very
        // likely to be true)
        q.push_back(0);
      }
    }
  }
  controller.setEncoderValues(q);
  controller.init(q);
  controller.running = true;

  size_t nextStep = 0;
  bool stepByStep = false;
  auto toogleStepByStep = [&]() {
    if(stepByStep)
    {
      stepByStep = false;
    }
    else
    {
      nextStep = 0;
      stepByStep = true;
    }
  };
  bool ticker_sync = true;
  bool ticker_run = true;
  mc_rtc::gui::StateBuilder * gui = get_gui(controller);
  if(gui)
  {
    gui->addElement({"Ticker"}, mc_rtc::gui::Button("Stop", [&ticker_run]() { ticker_run = false; }),
                    mc_rtc::gui::Checkbox(
                        "Sync with real-time", [&]() { return ticker_sync; }, [&]() { ticker_sync = !ticker_sync; }),
                    mc_rtc::gui::Checkbox(
                        "Step by step", [&]() { return stepByStep; }, [&]() { toogleStepByStep(); }));
    auto dt = controller.timestep();
    auto buttonText = [&](size_t n) {
      size_t n_ms = std::ceil(n * 1000 * dt);
      return "+" + std::to_string(n_ms) + "ms";
    };
    gui->addElement({"Ticker"}, mc_rtc::gui::ElementsStacking::Horizontal,
                    mc_rtc::gui::Button(buttonText(1), [&]() { nextStep = 1; }),
                    mc_rtc::gui::Button(buttonText(5), [&]() { nextStep = 5; }),
                    mc_rtc::gui::Button(buttonText(10), [&]() { nextStep = 10; }),
                    mc_rtc::gui::Button(buttonText(50), [&]() { nextStep = 20; }),
                    mc_rtc::gui::Button(buttonText(100), [&]() { nextStep = 100; }));
  }

  auto runController = [&]() {
    auto & mbc = controller.robot().mbc();
    const auto & rjo = controller.ref_joint_order();
    q.resize(rjo.size());
    size_t index = 0;
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      const auto & jn = rjo[i];
      if(controller.robot().hasJoint(jn))
      {
        for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)])
        {
          q[index] = qj;
          index++;
        }
      }
    }
    controller.setEncoderValues(q);
    controller.run();
  };
  auto updateGUI = [&]() {
    controller.running = false;
    controller.run();
    controller.running = true;
  };

  using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady, std::chrono::high_resolution_clock,
                                   std::chrono::steady_clock>;
  using duration_us = std::chrono::duration<double, std::micro>;
  auto dt = duration_us(1e6 * controller.timestep());

  // FIXME Catch ctrl^c to interrupt
  while(ticker_run)
  {
    auto now = clock::now();
    if(stepByStep)
    {
      if(nextStep > 0)
      {
        nextStep--;
        runController();
      }
      else
      {
        updateGUI();
      }
    }
    else
    {
      runController();
    }
    if(ticker_sync)
    {
      std::this_thread::sleep_until(now + dt);
    }
  }

  return 0;
}
