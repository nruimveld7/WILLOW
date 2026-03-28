#include "L298N.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace Willow {

void L298N::ApplyConfig(const DriverConfig& config) {
  config_ = static_cast<const L298NDriverConfig&>(config);
  isConfigured_ = HasValidConfig();
}

const DriverConfig& L298N::GetConfig() const { return config_; }

void L298N::Begin() {
  if (!isConfigured_) {
    isStarted_ = false;
    return;
  }
#ifdef ARDUINO
  pinMode(config_.enaPin, OUTPUT);
  pinMode(config_.pwmPin, OUTPUT);
  pinMode(config_.dirPin, OUTPUT);
  digitalWrite(config_.enaPin, LOW);
  digitalWrite(config_.dirPin, LOW);
  analogWrite(config_.pwmPin, 0);
#endif
  lastDirection_ = 0;
  isStarted_ = true;
}

void L298N::Stop() {
#ifdef ARDUINO
  digitalWrite(config_.enaPin, LOW);
  analogWrite(config_.pwmPin, 0);
#endif
  lastSignedOutputPercent_ = 0.0f;
  lastDirection_ = 0;
  isStarted_ = false;
}

void L298N::SetSignedOutput(float signedPercent) {
  if (!isStarted_) {
    return;
  }
  ApplyOutput(ClampSignedPercent(signedPercent));
}

bool L298N::GetCurrentAmps(float& currentAmps) const {
  currentAmps = 0.0f;
  return false;
}

bool L298N::GetCurrentSenseRaw(uint16_t& rightRaw, uint16_t& leftRaw) const {
  rightRaw = 0;
  leftRaw = 0;
  return false;
}

bool L298N::IsSoftLocked() const { return false; }

bool L298N::HasValidConfig() const {
  return IsConfiguredPin(config_.enaPin) && IsConfiguredPin(config_.pwmPin) &&
         IsConfiguredPin(config_.dirPin);
}

void L298N::ApplyOutput(float signedPercent) {
  const float magnitude = signedPercent >= 0.0f ? signedPercent : -signedPercent;
  int8_t desiredDirection = 0;
  if (magnitude > 0.0f) {
    desiredDirection = signedPercent >= 0.0f ? 1 : -1;
  }

#ifdef ARDUINO
  if (desiredDirection != 0 && lastDirection_ != 0 && desiredDirection != lastDirection_) {
    analogWrite(config_.pwmPin, 0);
    delayMicroseconds(config_.directionSwitchDeadtimeMicros);
  }

  if (desiredDirection == 0) {
    digitalWrite(config_.enaPin, LOW);
    analogWrite(config_.pwmPin, 0);
  } else {
    digitalWrite(config_.dirPin, desiredDirection > 0 ? HIGH : LOW);
    digitalWrite(config_.enaPin, HIGH);
    const int duty = SignedPercentToDuty(magnitude);
    analogWrite(config_.pwmPin, duty);
  }
#endif

  lastDirection_ = desiredDirection;
  lastSignedOutputPercent_ = signedPercent;
}

}  // namespace Willow
