#include "MotorDriver.h"

namespace Willow {

float MotorDriver::ClampSignedPercent(float value) {
  if (value > 100.0f) return 100.0f;
  if (value < -100.0f) return -100.0f;
  return value;
}

int MotorDriver::SignedPercentToDuty(float signedPercent) {
  const float clamped = ClampSignedPercent(signedPercent);
  const float magnitude = clamped >= 0.0f ? clamped : -clamped;
  return static_cast<int>((magnitude * 255.0f) / 100.0f);
}

bool MotorDriver::IsConfiguredPin(uint8_t pin) { return pin != 0xFF; }

}  // namespace Willow
