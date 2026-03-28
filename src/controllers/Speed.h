#pragma once

#include "../interfaces/controller/MotorController.h"

namespace Willow {

class Speed : public MotorController {
 public:
  Speed() = default;

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
  void SetFeedbackAvailable(bool hasFeedback);
  void SetMeasuredSpeedRpm(float measuredSpeedRpm);
  float GetMeasuredSpeedRpm() const;
  float GetTargetSpeedRpm() const;
  float GetTargetSpeedPercent() const;

 private:
  SpeedControllerConfig config_;
  float targetSpeedRpm_ = 0.0f;
  float targetSpeedPercent_ = 0.0f;
  float measuredSpeedRpm_ = 0.0f;
  PidState pidState_;
  float signedOutputPercent_ = 0.0f;
  bool hasFeedback_ = false;
  bool isStarted_ = false;
};

}  // namespace Willow
