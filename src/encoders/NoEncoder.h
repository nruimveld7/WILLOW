#pragma once

#include "../interfaces/encoder/MotorEncoder.h"

namespace Willow {

class NoEncoder : public MotorEncoder {
 public:
  NoEncoder() = default;

  void ApplyConfig(const EncoderConfig& config) override;
  const EncoderConfig& GetConfig() const override;

  void Begin() override;
  void OnPhaseAChange() override;
  void OnPhaseBChange() override;

 private:
  NoEncoderEncoderConfig config_;
};

}  // namespace Willow
