#pragma once

namespace Willow {

enum class MotorDriverType {
  BTS7960,
  L298N
};

enum class MotorEncoderType {
  None,
  Quadrature
};

enum class MotorControllerType {
  Position,
  Speed
};

struct MotorComponentSelection {
  MotorDriverType driver;
  MotorEncoderType encoder;
  MotorControllerType controller;
};

}  // namespace Willow
