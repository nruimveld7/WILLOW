#pragma once

#include "../interfaces/motor/MotorDriver.h"

namespace Willow {

class BTS7960 : public MotorDriver {
 public:
  BTS7960() = default;

  void ApplyConfig(const DriverConfig& config) override;
  const DriverConfig& GetConfig() const override;

  void Begin() override;
  void Stop() override;
  void SetSignedOutput(float signedPercent) override;
  bool GetCurrentAmps(float& currentAmps) const override;
  bool GetCurrentSenseRaw(uint16_t& rightRaw, uint16_t& leftRaw) const override;
  bool IsSoftLocked() const override;

 private:
  bool HasValidConfig() const;
  bool HasCurrentSensePins() const;
  bool IsCurrentLimitEnabled() const;
  void ApplyOutput(float signedPercent);
  void TriggerOverCurrent(float currentAmps);
  bool IsInLockout() const;

  BTS7960DriverConfig config_;
  float lastSignedOutputPercent_ = 0.0f;
  float lastCurrentAmps_ = 0.0f;
  unsigned long lockoutUntilMs_ = 0;
  bool isConfigured_ = false;
  bool isStarted_ = false;
};

}  // namespace Willow
