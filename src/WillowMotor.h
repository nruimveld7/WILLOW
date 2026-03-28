#pragma once

#include <stdint.h>

#include "common/Configs.h"
#include "interfaces/controller/MotorController.h"
#include "interfaces/encoder/MotorEncoder.h"
#include "interfaces/motor/MotorDriver.h"

namespace Willow {

class WillowMotor {
 public:
  WillowMotor() = default;
  ~WillowMotor();
  WillowMotor(const WillowMotor&) = delete;
  WillowMotor& operator=(const WillowMotor&) = delete;

  void SetDriverConfig(const DriverConfig& config);
  void SetEncoderConfig(const EncoderConfig& config);
  void SetControllerConfig(const ControllerConfig& config);

  const DriverConfig& GetDriverConfig() const;
  const EncoderConfig& GetEncoderConfig() const;
  const ControllerConfig& GetControllerConfig() const;

  bool Begin();
  bool ApplyConfig();
  void Stop();

  void Update();
  void OnControlInterrupt();
  void OnEncoderPhaseAChange();
  void OnEncoderPhaseBChange();

  bool SetSpeedRpm(float speedRpm);
  bool SetSpeedPercent(float speedPercent);
  bool SetPositionDegrees(float positionDegrees);
  bool SetPositionPercent(float positionPercent);
  long GetEncoderPulseCount() const;
  bool GetMeasuredSpeedRpm(float& speedRpm) const;
  bool GetMeasuredPositionDegrees(float& positionDegrees) const;
  bool GetTargetSpeedRpm(float& speedRpm) const;
  bool GetTargetSpeedPercent(float& speedPercent) const;
  bool GetTargetPositionDegrees(float& positionDegrees) const;
  bool GetTargetPositionPercent(float& positionPercent) const;
  bool GetControllerOutputPercent(float& signedPercent) const;
  bool GetDriverCurrentAmps(float& currentAmps) const;
  bool GetDriverCurrentSenseRaw(uint16_t& rightRaw, uint16_t& leftRaw) const;
  bool IsDriverSoftLocked() const;

 private:
  bool BuildComponents();
  bool ValidateCompatibility() const;
  bool IsDriverConfigComplete() const;
  bool IsEncoderConfigComplete() const;
  bool IsControllerConfigComplete() const;
  bool BuildDriver();
  bool BuildEncoder();
  bool BuildController();
  uint32_t GetConfiguredControlPeriodMicros() const;
  uint32_t ReadMicros() const;
  void UpdateControllerFeedback(uint32_t deltaMicros);
  bool IsFeedbackAvailable() const;
  float ComputeMeasuredSpeedRpm(long pulseDelta, uint32_t deltaMicros) const;
  float ComputeMeasuredPositionDegrees(long pulseCount) const;
  bool ShouldInterruptOnPhaseA() const;
  bool ShouldInterruptOnPhaseB() const;
  void QueueInterruptControlUpdate();
  uint8_t ConsumeInterruptControlUpdates();
  uint8_t GetMaxInterruptUpdatesPerLoop() const;

  DriverConfig* driverConfig_ = nullptr;
  EncoderConfig* encoderConfig_ = nullptr;
  ControllerConfig* controllerConfig_ = nullptr;

  MotorDriver* driver_ = nullptr;
  MotorEncoder* encoder_ = nullptr;
  MotorController* controller_ = nullptr;

  bool isStarted_ = false;
  bool hasLastUpdateMicros_ = false;
  uint32_t lastUpdateMicros_ = 0;
  volatile uint8_t pendingInterruptControlUpdates_ = 0;
};

}  // namespace Willow
