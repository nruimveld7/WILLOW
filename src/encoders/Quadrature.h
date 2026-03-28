#pragma once

#include "../interfaces/encoder/MotorEncoder.h"

namespace Willow {

class Quadrature : public MotorEncoder {
 public:
  Quadrature() = default;

  void ApplyConfig(const EncoderConfig& config) override;
  const EncoderConfig& GetConfig() const override;

  void Begin() override;
  void OnPhaseAChange() override;
  void OnPhaseBChange() override;

 private:
  void ApplyTransitionFromPins();
  void ApplySingleChannelStep(bool isPhaseAEdge);
  uint8_t ReadState() const;
  static int8_t DecodeTransition(uint8_t previous, uint8_t next);

  QuadratureEncoderConfig config_;
  uint8_t lastState_ = 0;
  bool isStarted_ = false;
};

}  // namespace Willow
