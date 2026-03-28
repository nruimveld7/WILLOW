#include "MotorController.h"

namespace Willow {

float MotorController::ClampSignedPercent(float value) {
  if (value > 100.0f) return 100.0f;
  if (value < -100.0f) return -100.0f;
  return value;
}

float MotorController::ClampAbs(float value, float maxAbs) {
  if (maxAbs <= 0.0f) {
    return value;
  }
  if (value > maxAbs) return maxAbs;
  if (value < -maxAbs) return -maxAbs;
  return value;
}

float MotorController::ComputePidOutput(float error, float kp, float ki, float kd,
                                        uint32_t deltaMicros, float outputLimitAbs,
                                        PidState& state) {
  const float dtSeconds =
      deltaMicros > 0 ? static_cast<float>(deltaMicros) / 1000000.0f : 0.0f;

  const float derivative =
      dtSeconds > 0.0f ? (error - state.lastError) / dtSeconds : 0.0f;

  float candidateIntegral = state.integral;
  if (dtSeconds > 0.0f) {
    candidateIntegral += error * dtSeconds;
  }

  const float pTerm = kp * error;
  const float dTerm = kd * derivative;
  const float candidateOutput = pTerm + (ki * candidateIntegral) + dTerm;
  const float clampedCandidate = ClampAbs(candidateOutput, outputLimitAbs);
  const bool saturated = candidateOutput != clampedCandidate;
  const bool drivesFurtherIntoSaturation =
      (clampedCandidate > 0.0f && error > 0.0f) ||
      (clampedCandidate < 0.0f && error < 0.0f);

  if (!(saturated && drivesFurtherIntoSaturation)) {
    state.integral = candidateIntegral;
  }

  state.lastError = error;
  return ClampAbs(pTerm + (ki * state.integral) + dTerm, outputLimitAbs);
}

uint32_t MotorController::GetControlPeriodMicros(const ControllerConfig& config) {
  if (config.GetType() == MotorControllerType::Speed) {
    return static_cast<const SpeedControllerConfig&>(config).controlPeriodMicros;
  }
  return static_cast<const PositionControllerConfig&>(config).controlPeriodMicros;
}

}  // namespace Willow
