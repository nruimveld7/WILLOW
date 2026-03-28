#include "Speed.h"

#include <math.h>

namespace Willow {

void Speed::ApplyConfig(const ControllerConfig& config) {
  config_ = static_cast<const SpeedControllerConfig&>(config);
  targetSpeedPercent_ = ClampSignedPercent(targetSpeedPercent_);
}

const ControllerConfig& Speed::GetConfig() const { return config_; }

void Speed::Begin() { isStarted_ = true; }

void Speed::Stop() {
  isStarted_ = false;
  signedOutputPercent_ = 0.0f;
  pidState_ = PidState{};
}

void Speed::Update(uint32_t deltaMicros) {
  if (!isStarted_) {
    return;
  }

  if (config_.zeroSpeedHoldDeadbandRpm > 0.0f && targetSpeedRpm_ == 0.0f &&
      fabsf(measuredSpeedRpm_) <= config_.zeroSpeedHoldDeadbandRpm) {
    signedOutputPercent_ = 0.0f;
    pidState_ = PidState{};
    return;
  }

  if (!hasFeedback_ || config_.speedReferenceRpm <= 0.0f) {
    signedOutputPercent_ = ClampSignedPercent(targetSpeedPercent_);
    return;
  }

  const float error = targetSpeedRpm_ - measuredSpeedRpm_;
  const float outputPercent = ComputePidOutput(
      error, config_.kp, config_.ki, config_.kd, deltaMicros, 100.0f, pidState_);
  signedOutputPercent_ = ClampSignedPercent(outputPercent);
}

void Speed::OnControlInterrupt() {
  Update(GetControlPeriodMicros(config_));
}

bool Speed::SetSpeedRpm(float speedRpm) {
  targetSpeedRpm_ = ClampAbs(speedRpm, config_.maxSpeedRpm);
  if (config_.speedReferenceRpm <= 0.0f) {
    targetSpeedPercent_ = 0.0f;
    return false;
  }
  targetSpeedPercent_ =
      ClampSignedPercent((targetSpeedRpm_ / config_.speedReferenceRpm) * 100.0f);
  return true;
}

bool Speed::SetSpeedPercent(float speedPercent) {
  targetSpeedPercent_ = ClampSignedPercent(speedPercent);
  if (config_.speedReferenceRpm <= 0.0f) {
    targetSpeedRpm_ = 0.0f;
    return !hasFeedback_;
  }
  targetSpeedRpm_ = (targetSpeedPercent_ / 100.0f) * config_.speedReferenceRpm;
  return true;
}

bool Speed::SetPositionDegrees(float /*positionDegrees*/) { return false; }

bool Speed::SetPositionPercent(float /*positionPercent*/) { return false; }

float Speed::GetSignedOutputPercent() const { return signedOutputPercent_; }

void Speed::SetFeedbackAvailable(bool hasFeedback) { hasFeedback_ = hasFeedback; }

void Speed::SetMeasuredSpeedRpm(float measuredSpeedRpm) {
  measuredSpeedRpm_ = measuredSpeedRpm;
}

float Speed::GetMeasuredSpeedRpm() const { return measuredSpeedRpm_; }

float Speed::GetTargetSpeedRpm() const { return targetSpeedRpm_; }

float Speed::GetTargetSpeedPercent() const { return targetSpeedPercent_; }

}  // namespace Willow
