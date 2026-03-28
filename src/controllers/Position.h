#pragma once

#include "../interfaces/controller/MotorController.h"

namespace Willow {

class Position : public MotorController {
 public:
  Position() = default;

  void ApplyConfig(const ControllerConfig& config) override;
  const ControllerConfig& GetConfig() const override;

  void Begin() override;
  void Stop() override;
  void Update(uint32_t deltaMicros) override;
  void OnControlInterrupt() override;

  bool SetSpeedRpm(float speedRpm) override;
  bool SetSpeedPercent(float speedPercent) override;
  bool SetPositionDegrees(float positionDegrees) override;
  bool SetPositionPercent(float positionPercent) override;

  float GetSignedOutputPercent() const override;
  void SetMeasuredPositionDegrees(float measuredPositionDegrees);
  float GetMeasuredPositionDegrees() const;
  float GetTargetPositionDegrees() const;
  float GetTargetPositionPercent() const;

 private:
  float NormalizeDegrees(float value) const;
  float ComputeShortestError(float targetDegrees, float measuredDegrees) const;

  PositionControllerConfig config_;
  float targetPositionDegrees_ = 0.0f;
  float targetPositionPercent_ = 0.0f;
  float measuredPositionDegrees_ = 0.0f;
  PidState pidState_;
  float signedOutputPercent_ = 0.0f;
  bool isStarted_ = false;
};

}  // namespace Willow
