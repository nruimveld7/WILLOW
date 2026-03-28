#include "NoEncoder.h"

namespace Willow {

void NoEncoder::ApplyConfig(const EncoderConfig& config) {
  config_ = static_cast<const NoEncoderEncoderConfig&>(config);
}

const EncoderConfig& NoEncoder::GetConfig() const { return config_; }

void NoEncoder::Begin() {}

void NoEncoder::OnPhaseAChange() {}

void NoEncoder::OnPhaseBChange() {}

}  // namespace Willow
