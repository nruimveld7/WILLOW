#include "Position.h"

#include <math.h>

namespace Willow {

namespace {
float NormalizePercent(float value) {
  // Map to [-100, +100] with modulo wrapping.
  float wrapped = fmodf(value, 200.0f);
  if (wrapped > 100.0f) {
    wrapped -= 200.0f;
  } else if (wrapped < -100.0f) {
    wrapped += 200.0f;
  }
  return wrapped;
}
}  // namespace

void Position::ApplyConfig(const ControllerConfig& config) {
  config_ = static_cast<const PositionControllerConfig&>(config);
  targetPositionPercent_ = NormalizePercent(targetPositionPercent_);
  targetPositionDegrees_ =
      (targetPositionPercent_ / 100.0f) * config_.positionReferenceDegrees;
}

const ControllerConfig& Position::GetConfig() const { return config_; }

void Position::Begin() { isStarted_ = true; }

void Position::Stop() {
  isStarted_ = false;
  signedOutputPercent_ = 0.0f;
  pidState_ = PidState{};
}

void Position::Update(uint32_t deltaMicros) {
  if (!isStarted_) {
    return;
  }

  const float error =
      ComputeShortestError(targetPositionDegrees_, measuredPositionDegrees_);
  if (config_.holdDeadbandDegrees > 0.0f &&
      fabsf(error) <= config_.holdDeadbandDegrees) {
    signedOutputPercent_ = 0.0f;
    pidState_ = PidState{};
    return;
  }

  const float outputPercent = ComputePidOutput(
      error, config_.kp, config_.ki, config_.kd, deltaMicros, 100.0f, pidState_);
  signedOutputPercent_ = ClampSignedPercent(outputPercent);
}

void Position::OnControlInterrupt() {
  Update(GetControlPeriodMicros(config_));
}

bool Position::SetSpeedRpm(float /*speedRpm*/) { return false; }

bool Position::SetSpeedPercent(float /*speedPercent*/) { return false; }

bool Position::SetPositionDegrees(float positionDegrees) {
  targetPositionDegrees_ = NormalizeDegrees(positionDegrees);
  if (config_.positionReferenceDegrees == 0.0f) {
    targetPositionPercent_ = 0.0f;
    return false;
  }
  targetPositionPercent_ = NormalizePercent(
      (targetPositionDegrees_ / config_.positionReferenceDegrees) * 100.0f);
  return true;
}

bool Position::SetPositionPercent(float positionPercent) {
  targetPositionPercent_ = NormalizePercent(positionPercent);
  targetPositionDegrees_ =
      (targetPositionPercent_ / 100.0f) * config_.positionReferenceDegrees;
  return true;
}

float Position::GetSignedOutputPercent() const { return signedOutputPercent_; }

void Position::SetMeasuredPositionDegrees(float measuredPositionDegrees) {
  measuredPositionDegrees_ = measuredPositionDegrees;
}

float Position::GetMeasuredPositionDegrees() const {
  return measuredPositionDegrees_;
}

float Position::GetTargetPositionDegrees() const { return targetPositionDegrees_; }

float Position::GetTargetPositionPercent() const { return targetPositionPercent_; }

float Position::NormalizeDegrees(float value) const {
  if (config_.positionReferenceDegrees <= 0.0f) {
    return value;
  }
  const float span = config_.positionReferenceDegrees * 2.0f;
  float wrapped = fmodf(value, span);
  if (wrapped > config_.positionReferenceDegrees) {
    wrapped -= span;
  } else if (wrapped < -config_.positionReferenceDegrees) {
    wrapped += span;
  }
  return wrapped;
}

float Position::ComputeShortestError(float targetDegrees, float measuredDegrees) const {
  return NormalizeDegrees(targetDegrees - NormalizeDegrees(measuredDegrees));
}

}  // namespace Willow
