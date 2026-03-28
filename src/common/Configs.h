#pragma once

#include <stdint.h>

#include "ComponentSelection.h"

namespace Willow {

typedef void (*OverCurrentCallback)(float currentAmps);

enum class QuadratureInterruptMode {
  Manual,
  PhaseA,
  PhaseB,
  PhaseAB
};

class DriverConfig {
 public:
  virtual ~DriverConfig() = default;
  virtual MotorDriverType GetType() const = 0;
  virtual DriverConfig* Clone() const = 0;
};

class EncoderConfig {
 public:
  virtual ~EncoderConfig() = default;
  virtual MotorEncoderType GetType() const = 0;
  virtual EncoderConfig* Clone() const = 0;
};

class ControllerConfig {
 public:
  virtual ~ControllerConfig() = default;
  virtual MotorControllerType GetType() const = 0;
  virtual ControllerConfig* Clone() const = 0;
};

struct BTS7960DriverConfig : public DriverConfig {
  uint8_t pwmLeftPin = 0xFF;
  uint8_t pwmRightPin = 0xFF;
  uint8_t currentSenseLeftPin = 0xFF;
  uint8_t currentSenseRightPin = 0xFF;
  uint8_t enableLeftPin = 0xFF;
  uint8_t enableRightPin = 0xFF;
  float adcReferenceVolts = 5.0f;
  uint16_t adcResolutionCounts = 1023;
  float currentSenseVoltsPerAmp = 1.0f;
  float currentLimitAmps = 0.0f;
  uint32_t currentLimitLockoutMillis = 250;
  bool softLockOnOverCurrent = true;
  OverCurrentCallback onOverCurrent = nullptr;

  MotorDriverType GetType() const override { return MotorDriverType::BTS7960; }
  DriverConfig* Clone() const override { return new BTS7960DriverConfig(*this); }
};

struct L298NDriverConfig : public DriverConfig {
  uint8_t enaPin = 0xFF;
  uint8_t pwmPin = 0xFF;
  uint8_t dirPin = 0xFF;
  uint16_t directionSwitchDeadtimeMicros = 500;

  MotorDriverType GetType() const override { return MotorDriverType::L298N; }
  DriverConfig* Clone() const override { return new L298NDriverConfig(*this); }
};

struct QuadratureEncoderConfig : public EncoderConfig {
  uint8_t pinA = 0xFF;
  uint8_t pinB = 0xFF;
  uint16_t pulsesPerRevolution = 20;
  bool useInternalPullups = false;
  QuadratureInterruptMode interruptMode = QuadratureInterruptMode::Manual;

  MotorEncoderType GetType() const override {
    return MotorEncoderType::Quadrature;
  }
  EncoderConfig* Clone() const override {
    return new QuadratureEncoderConfig(*this);
  }
};

struct NoEncoderEncoderConfig : public EncoderConfig {
  MotorEncoderType GetType() const override { return MotorEncoderType::None; }
  EncoderConfig* Clone() const override { return new NoEncoderEncoderConfig(*this); }
};

struct SpeedControllerConfig : public ControllerConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float maxSpeedRpm = 0.0f;
  float speedReferenceRpm = 0.0f;
  float zeroSpeedHoldDeadbandRpm = 0.0f;
  uint32_t controlPeriodMicros = 10000;
  uint8_t maxInterruptUpdatesPerLoop = 8;

  MotorControllerType GetType() const override {
    return MotorControllerType::Speed;
  }
  ControllerConfig* Clone() const override {
    return new SpeedControllerConfig(*this);
  }
};

struct PositionControllerConfig : public ControllerConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float positionReferenceDegrees = 360.0f;
  float holdDeadbandDegrees = 0.0f;
  uint32_t controlPeriodMicros = 10000;
  uint8_t maxInterruptUpdatesPerLoop = 8;

  MotorControllerType GetType() const override {
    return MotorControllerType::Position;
  }
  ControllerConfig* Clone() const override {
    return new PositionControllerConfig(*this);
  }
};

}  // namespace Willow
