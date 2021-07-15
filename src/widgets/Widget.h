#pragma once

/** Helper header to have all the stuff available in widgets */

#include "../McRtcGui.h"
#include "../mc_rtc-imgui/Widget.h"

#include "utils.h"

namespace mc_rtc::magnum
{

struct Widget : public mc_rtc::imgui::Widget
{
  Widget(Client & client, const ElementId & id, McRtcGui & gui) : mc_rtc::imgui::Widget(client, id), gui_(gui) {}

protected:
  McRtcGui & gui_;
};

} // namespace mc_rtc::magnum
