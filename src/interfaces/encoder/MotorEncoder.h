#pragma once

#include "../../common/Configs.h"

namespace Willow {

class MotorEncoder {
 public:
  virtual ~MotorEncoder() = default;

  virtual void ApplyConfig(const EncoderConfig& config) = 0;
  virtual const EncoderConfig& GetConfig() const = 0;

  virtual void Begin() = 0;
  virtual void OnPhaseAChange() = 0;
  virtual void OnPhaseBChange() = 0;
  virtual long ConsumePulseDelta();
  virtual long GetPulseCount() const;

 protected:
  void ResetPulseTracking();
  void AddPulseDelta(long delta);

 private:
  volatile long pulseCount_ = 0;
  long lastConsumedPulseCount_ = 0;
};

}  // namespace Willow
