#pragma once

#include <stdint.h>

#include "../../common/Configs.h"

namespace Willow {

class MotorController {
 public:
  struct PidState {
    float integral = 0.0f;
    float lastError = 0.0f;
  };

  virtual ~MotorController() = default;

  virtual void ApplyConfig(const ControllerConfig& config) = 0;
  virtual const ControllerConfig& GetConfig() const = 0;

  virtual void Begin() = 0;
  virtual void Stop() = 0;
  virtual void Update(uint32_t deltaMicros) = 0;
  virtual void OnControlInterrupt() = 0;

  virtual bool SetSpeedRpm(float speedRpm) = 0;
  virtual bool SetSpeedPercent(float speedPercent) = 0;
  virtual bool SetPositionDegrees(float positionDegrees) = 0;
  virtual bool SetPositionPercent(float positionPercent) = 0;

  virtual float GetSignedOutputPercent() const = 0;

 protected:
  static float ClampSignedPercent(float value);
  static float ClampAbs(float value, float maxAbs);
  static float ComputePidOutput(float error, float kp, float ki, float kd,
                                uint32_t deltaMicros, float outputLimitAbs,
                                PidState& state);
  static uint32_t GetControlPeriodMicros(const ControllerConfig& config);
};

}  // namespace Willow
