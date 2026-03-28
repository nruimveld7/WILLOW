#include "BTS7960.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace Willow {

void BTS7960::ApplyConfig(const DriverConfig& config) {
  config_ = static_cast<const BTS7960DriverConfig&>(config);
  isConfigured_ = HasValidConfig();
}

const DriverConfig& BTS7960::GetConfig() const { return config_; }

void BTS7960::Begin() {
  if (!isConfigured_) {
    isStarted_ = false;
    return;
  }
#ifdef ARDUINO
  pinMode(config_.pwmLeftPin, OUTPUT);
  pinMode(config_.pwmRightPin, OUTPUT);
  pinMode(config_.enableLeftPin, OUTPUT);
  pinMode(config_.enableRightPin, OUTPUT);
  if (HasCurrentSensePins()) {
    pinMode(config_.currentSenseLeftPin, INPUT);
    pinMode(config_.currentSenseRightPin, INPUT);
  }
  digitalWrite(config_.enableLeftPin, LOW);
  digitalWrite(config_.enableRightPin, LOW);
  analogWrite(config_.pwmLeftPin, 0);
  analogWrite(config_.pwmRightPin, 0);
#endif
  isStarted_ = true;
}

void BTS7960::Stop() {
#ifdef ARDUINO
  digitalWrite(config_.enableLeftPin, LOW);
  digitalWrite(config_.enableRightPin, LOW);
  analogWrite(config_.pwmLeftPin, 0);
  analogWrite(config_.pwmRightPin, 0);
#endif
  lastSignedOutputPercent_ = 0.0f;
  lastCurrentAmps_ = 0.0f;
  lockoutUntilMs_ = 0;
  isStarted_ = false;
}

void BTS7960::SetSignedOutput(float signedPercent) {
  if (!isStarted_) {
    return;
  }

  float currentAmps = 0.0f;
  const bool hasCurrentReading = GetCurrentAmps(currentAmps);
  if (hasCurrentReading) {
    lastCurrentAmps_ = currentAmps;
  }

  if (IsInLockout()) {
    ApplyOutput(0.0f);
    return;
  }

  if (hasCurrentReading && IsCurrentLimitEnabled() &&
      currentAmps >= config_.currentLimitAmps) {
    TriggerOverCurrent(currentAmps);
    ApplyOutput(0.0f);
    return;
  }

  ApplyOutput(ClampSignedPercent(signedPercent));
}

bool BTS7960::GetCurrentAmps(float& currentAmps) const {
  if (!HasCurrentSensePins() || config_.adcResolutionCounts == 0 ||
      config_.currentSenseVoltsPerAmp <= 0.0f) {
    currentAmps = 0.0f;
    return false;
  }

#ifdef ARDUINO
  const int rawLeft = analogRead(config_.currentSenseLeftPin);
  const int rawRight = analogRead(config_.currentSenseRightPin);
  const int raw = rawLeft > rawRight ? rawLeft : rawRight;
  const float volts =
      (static_cast<float>(raw) * config_.adcReferenceVolts) /
      static_cast<float>(config_.adcResolutionCounts);
  currentAmps = volts / config_.currentSenseVoltsPerAmp;
  return true;
#else
  currentAmps = 0.0f;
  return false;
#endif
}

bool BTS7960::GetCurrentSenseRaw(uint16_t& rightRaw, uint16_t& leftRaw) const {
  rightRaw = 0;
  leftRaw = 0;
  if (!HasCurrentSensePins()) {
    return false;
  }

#ifdef ARDUINO
  rightRaw = static_cast<uint16_t>(analogRead(config_.currentSenseRightPin));
  leftRaw = static_cast<uint16_t>(analogRead(config_.currentSenseLeftPin));
  return true;
#else
  return false;
#endif
}

bool BTS7960::IsSoftLocked() const { return IsInLockout(); }

void BTS7960::ApplyOutput(float signedPercent) {
  const float clamped = ClampSignedPercent(signedPercent);
#ifdef ARDUINO
  const int duty = SignedPercentToDuty(clamped);
  digitalWrite(config_.enableLeftPin, HIGH);
  digitalWrite(config_.enableRightPin, HIGH);
  if (clamped >= 0.0f) {
    analogWrite(config_.pwmLeftPin, duty);
    analogWrite(config_.pwmRightPin, 0);
  } else {
    analogWrite(config_.pwmLeftPin, 0);
    analogWrite(config_.pwmRightPin, duty);
  }
#endif
  lastSignedOutputPercent_ = clamped;
}

void BTS7960::TriggerOverCurrent(float currentAmps) {
  if (config_.onOverCurrent) {
    config_.onOverCurrent(currentAmps);
  }

#ifdef ARDUINO
  if (config_.softLockOnOverCurrent) {
    lockoutUntilMs_ = millis() + config_.currentLimitLockoutMillis;
  }
#endif
}

bool BTS7960::HasCurrentSensePins() const {
  return IsConfiguredPin(config_.currentSenseLeftPin) &&
         IsConfiguredPin(config_.currentSenseRightPin);
}

bool BTS7960::IsCurrentLimitEnabled() const {
  return config_.currentLimitAmps > 0.0f;
}

bool BTS7960::IsInLockout() const {
#ifdef ARDUINO
  if (lockoutUntilMs_ == 0) {
    return false;
  }
  const long remaining = static_cast<long>(lockoutUntilMs_ - millis());
  return remaining > 0;
#else
  return false;
#endif
}

bool BTS7960::HasValidConfig() const {
  return IsConfiguredPin(config_.pwmLeftPin) &&
         IsConfiguredPin(config_.pwmRightPin) &&
         IsConfiguredPin(config_.enableLeftPin) &&
         IsConfiguredPin(config_.enableRightPin);
}

}  // namespace Willow
