#include "Quadrature.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace Willow {

void Quadrature::ApplyConfig(const EncoderConfig& config) {
  config_ = static_cast<const QuadratureEncoderConfig&>(config);
}

const EncoderConfig& Quadrature::GetConfig() const { return config_; }

void Quadrature::Begin() {
  isStarted_ = true;
  ResetPulseTracking();
#ifdef ARDUINO
  const uint8_t pinModeValue = config_.useInternalPullups ? INPUT_PULLUP : INPUT;
  pinMode(config_.pinA, pinModeValue);
  pinMode(config_.pinB, pinModeValue);
#endif
  lastState_ = ReadState();
}

void Quadrature::OnPhaseAChange() {
  if (isStarted_) {
    if (config_.interruptMode == QuadratureInterruptMode::PhaseB) {
      return;
    }
    if (config_.interruptMode == QuadratureInterruptMode::PhaseA) {
      ApplySingleChannelStep(true);
      return;
    }
    ApplyTransitionFromPins();
  }
}

void Quadrature::OnPhaseBChange() {
  if (isStarted_) {
    if (config_.interruptMode == QuadratureInterruptMode::PhaseA) {
      return;
    }
    if (config_.interruptMode == QuadratureInterruptMode::PhaseB) {
      ApplySingleChannelStep(false);
      return;
    }
    ApplyTransitionFromPins();
  }
}

void Quadrature::Poll() {
  if (!isStarted_) {
    return;
  }
  ApplyTransitionFromPins();
}

void Quadrature::ApplyTransitionFromPins() {
  const uint8_t nextState = ReadState();
  AddPulseDelta(DecodeTransition(lastState_, nextState));
  lastState_ = nextState;
}

void Quadrature::ApplySingleChannelStep(bool isPhaseAEdge) {
  const uint8_t nextState = ReadState();
  const bool previousAHigh = (lastState_ & 0x2u) != 0;
  const bool previousBHigh = (lastState_ & 0x1u) != 0;
  const bool nextAHigh = (nextState & 0x2u) != 0;
  const bool nextBHigh = (nextState & 0x1u) != 0;

  int8_t step = 0;
  if (isPhaseAEdge) {
    if (previousAHigh != nextAHigh) {
      const bool isRisingEdge = !previousAHigh && nextAHigh;
      step = isRisingEdge ? (nextBHigh ? -1 : 1) : (nextBHigh ? 1 : -1);
    }
  } else {
    if (previousBHigh != nextBHigh) {
      const bool isRisingEdge = !previousBHigh && nextBHigh;
      step = isRisingEdge ? (nextAHigh ? 1 : -1) : (nextAHigh ? -1 : 1);
    }
  }

  AddPulseDelta(step);
  lastState_ = nextState;
}

uint8_t Quadrature::ReadState() const {
#ifdef ARDUINO
  const uint8_t a = digitalRead(config_.pinA) ? 1u : 0u;
  const uint8_t b = digitalRead(config_.pinB) ? 1u : 0u;
  return static_cast<uint8_t>((a << 1) | b);
#else
  return lastState_;
#endif
}

int8_t Quadrature::DecodeTransition(uint8_t previous, uint8_t next) {
  static const int8_t transitionTable[16] = {
      0,  -1, 1,  0,   // 00 -> 00/01/10/11
      1,  0,  0,  -1,  // 01 -> 00/01/10/11
      -1, 0,  0,  1,   // 10 -> 00/01/10/11
      0,  1,  -1, 0    // 11 -> 00/01/10/11
  };
  const uint8_t index =
      static_cast<uint8_t>(((previous & 0x3u) << 2) | (next & 0x3u));
  return transitionTable[index];
}

}  // namespace Willow
