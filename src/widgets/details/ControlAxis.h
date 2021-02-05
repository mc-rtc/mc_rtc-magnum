#pragma once

enum class ControlAxis
{
  TX = (1u << 0),
  TY = (1u << 1),
  TZ = (1u << 2),
  RX = (1u << 3),
  RY = (1u << 4),
  RZ = (1u << 5),
  TRANSLATION = TX | TY | TZ,
  ROTATION = RX | RY | RZ,
  XYTHETA = TX | TY | RZ,
  XYZTHETA = TX | TY | TZ | RZ,
  ALL = TRANSLATION | ROTATION
};
