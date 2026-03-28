#pragma once

#include "../interfaces/motor/MotorDriver.h"

namespace Willow {

class L298N : public MotorDriver {
 public:
  L298N() = default;

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
  void ApplyOutput(float signedPercent);

  L298NDriverConfig config_;
  float lastSignedOutputPercent_ = 0.0f;
  int8_t lastDirection_ = 0;
  bool isConfigured_ = false;
  bool isStarted_ = false;
};

}  // namespace Willow
