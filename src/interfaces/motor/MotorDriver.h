#pragma once

#include "../../common/Configs.h"

namespace Willow {

class MotorDriver {
 public:
  virtual ~MotorDriver() = default;

  virtual void ApplyConfig(const DriverConfig& config) = 0;
  virtual const DriverConfig& GetConfig() const = 0;

  virtual void Begin() = 0;
  virtual void Stop() = 0;
  virtual void SetSignedOutput(float signedPercent) = 0;
  virtual bool GetCurrentAmps(float& currentAmps) const = 0;
  virtual bool GetCurrentSenseRaw(uint16_t& rightRaw, uint16_t& leftRaw) const = 0;
  virtual bool IsSoftLocked() const = 0;

 protected:
  static float ClampSignedPercent(float value);
  static int SignedPercentToDuty(float signedPercent);
  static bool IsConfiguredPin(uint8_t pin);
};

}  // namespace Willow
