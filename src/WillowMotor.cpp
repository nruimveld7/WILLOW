#include "WillowMotor.h"

#ifdef ARDUINO
#include <Arduino.h>
#ifdef __AVR__
#include <avr/interrupt.h>
#endif
#else
#include <chrono>
#endif

#include "controllers/Position.h"
#include "controllers/Speed.h"
#include "drivers/BTS7960.h"
#include "drivers/L298N.h"
#include "encoders/NoEncoder.h"
#include "encoders/Quadrature.h"

namespace Willow {

namespace {
struct UnsetDriverConfig : public DriverConfig {
  MotorDriverType GetType() const override {
    return static_cast<MotorDriverType>(0xFF);
  }
  DriverConfig* Clone() const override { return new UnsetDriverConfig(*this); }
};

struct UnsetEncoderConfig : public EncoderConfig {
  MotorEncoderType GetType() const override {
    return static_cast<MotorEncoderType>(0xFF);
  }
  EncoderConfig* Clone() const override { return new UnsetEncoderConfig(*this); }
};

struct UnsetControllerConfig : public ControllerConfig {
  MotorControllerType GetType() const override {
    return static_cast<MotorControllerType>(0xFF);
  }
  ControllerConfig* Clone() const override {
    return new UnsetControllerConfig(*this);
  }
};

const UnsetDriverConfig kUnsetDriverConfig;
const UnsetEncoderConfig kUnsetEncoderConfig;
const UnsetControllerConfig kUnsetControllerConfig;

#ifdef __AVR__
uint8_t EnterCritical() {
  const uint8_t previousSreg = SREG;
  cli();
  return previousSreg;
}

void ExitCritical(uint8_t previousSreg) { SREG = previousSreg; }
#elif defined(ARDUINO)
uint8_t EnterCritical() {
  noInterrupts();
  return 0;
}

void ExitCritical(uint8_t /*unused*/) { interrupts(); }
#else
uint8_t EnterCritical() { return 0; }
void ExitCritical(uint8_t /*unused*/) {}
#endif
}

WillowMotor::~WillowMotor() {
  delete driver_;
  delete encoder_;
  delete controller_;
  delete driverConfig_;
  delete encoderConfig_;
  delete controllerConfig_;
}

void WillowMotor::SetDriverConfig(const DriverConfig& config) {
  delete driverConfig_;
  driverConfig_ = config.Clone();
}

void WillowMotor::SetEncoderConfig(const EncoderConfig& config) {
  delete encoderConfig_;
  encoderConfig_ = config.Clone();
}

void WillowMotor::SetControllerConfig(const ControllerConfig& config) {
  delete controllerConfig_;
  controllerConfig_ = config.Clone();
}

const DriverConfig& WillowMotor::GetDriverConfig() const {
  if (driver_) {
    return driver_->GetConfig();
  }
  if (driverConfig_) {
    return *driverConfig_;
  }
  return kUnsetDriverConfig;
}

const EncoderConfig& WillowMotor::GetEncoderConfig() const {
  if (encoder_) {
    return encoder_->GetConfig();
  }
  if (encoderConfig_) {
    return *encoderConfig_;
  }
  return kUnsetEncoderConfig;
}

const ControllerConfig& WillowMotor::GetControllerConfig() const {
  if (controller_) {
    return controller_->GetConfig();
  }
  if (controllerConfig_) {
    return *controllerConfig_;
  }
  return kUnsetControllerConfig;
}

bool WillowMotor::Begin() {
  if (!ApplyConfig()) {
    return false;
  }
  driver_->Begin();
  encoder_->Begin();
  controller_->Begin();
  isStarted_ = true;
  hasLastUpdateMicros_ = false;
  pendingInterruptControlUpdates_ = 0;
  return true;
}

bool WillowMotor::ApplyConfig() {
  const bool wasStarted = isStarted_;
  if (wasStarted) {
    Stop();
  }

  if (!BuildComponents()) {
    return false;
  }

  if (wasStarted) {
    driver_->Begin();
    encoder_->Begin();
    controller_->Begin();
    isStarted_ = true;
    hasLastUpdateMicros_ = false;
  }

  return true;
}

void WillowMotor::Stop() {
  if (controller_) {
    controller_->Stop();
  }
  if (driver_) {
    driver_->Stop();
  }
  isStarted_ = false;
  hasLastUpdateMicros_ = false;
  pendingInterruptControlUpdates_ = 0;
}

void WillowMotor::Update() {
  if (!isStarted_ || !controller_ || !driver_) {
    return;
  }

  if (encoder_ && encoderConfig_ &&
      encoderConfig_->GetType() == MotorEncoderType::Quadrature) {
    const QuadratureEncoderConfig& quadratureConfig =
        static_cast<const QuadratureEncoderConfig&>(*encoderConfig_);
    if (quadratureConfig.interruptMode == QuadratureInterruptMode::Manual) {
      static_cast<Quadrature*>(encoder_)->Poll();
    }
  }

  const uint32_t now = ReadMicros();
  const uint32_t controlPeriodMicros = GetConfiguredControlPeriodMicros();
  ConsumeInterruptControlUpdates();

  if (!hasLastUpdateMicros_) {
    lastUpdateMicros_ = now;
    hasLastUpdateMicros_ = true;
    return;
  }

  const uint32_t delta = now - lastUpdateMicros_;
  if (delta < controlPeriodMicros) {
    return;
  }

  lastUpdateMicros_ = now;
  UpdateControllerFeedback(delta);
  controller_->Update(delta);
  driver_->SetSignedOutput(controller_->GetSignedOutputPercent());
}

void WillowMotor::OnControlInterrupt() {
  if (!isStarted_ || !controller_ || !driver_) {
    return;
  }
  QueueInterruptControlUpdate();
}

void WillowMotor::OnEncoderPhaseAChange() {
  if (!isStarted_ || !encoder_) {
    return;
  }
  encoder_->OnPhaseAChange();
  if (ShouldInterruptOnPhaseA()) {
    QueueInterruptControlUpdate();
  }
}

void WillowMotor::OnEncoderPhaseBChange() {
  if (!isStarted_ || !encoder_) {
    return;
  }
  encoder_->OnPhaseBChange();
  if (ShouldInterruptOnPhaseB()) {
    QueueInterruptControlUpdate();
  }
}

bool WillowMotor::SetSpeedRpm(float speedRpm) {
  return controller_ && controller_->SetSpeedRpm(speedRpm);
}

bool WillowMotor::SetSpeedPercent(float speedPercent) {
  return controller_ && controller_->SetSpeedPercent(speedPercent);
}

bool WillowMotor::SetPositionDegrees(float positionDegrees) {
  return controller_ && controller_->SetPositionDegrees(positionDegrees);
}

bool WillowMotor::SetPositionPercent(float positionPercent) {
  return controller_ && controller_->SetPositionPercent(positionPercent);
}

long WillowMotor::GetEncoderPulseCount() const {
  return encoder_ ? encoder_->GetPulseCount() : 0;
}

bool WillowMotor::GetMeasuredSpeedRpm(float& speedRpm) const {
  if (!controller_ || !controllerConfig_ ||
      controllerConfig_->GetType() != MotorControllerType::Speed) {
    speedRpm = 0.0f;
    return false;
  }

  speedRpm = static_cast<const Speed*>(controller_)->GetMeasuredSpeedRpm();
  return true;
}

bool WillowMotor::GetMeasuredPositionDegrees(float& positionDegrees) const {
  if (!controller_ || !controllerConfig_ ||
      controllerConfig_->GetType() != MotorControllerType::Position) {
    positionDegrees = 0.0f;
    return false;
  }

  positionDegrees =
      static_cast<const Position*>(controller_)->GetMeasuredPositionDegrees();
  return true;
}

bool WillowMotor::GetTargetSpeedRpm(float& speedRpm) const {
  if (!controller_ || !controllerConfig_ ||
      controllerConfig_->GetType() != MotorControllerType::Speed) {
    speedRpm = 0.0f;
    return false;
  }

  speedRpm = static_cast<const Speed*>(controller_)->GetTargetSpeedRpm();
  return true;
}

bool WillowMotor::GetTargetSpeedPercent(float& speedPercent) const {
  if (!controller_ || !controllerConfig_ ||
      controllerConfig_->GetType() != MotorControllerType::Speed) {
    speedPercent = 0.0f;
    return false;
  }

  speedPercent = static_cast<const Speed*>(controller_)->GetTargetSpeedPercent();
  return true;
}

bool WillowMotor::GetTargetPositionDegrees(float& positionDegrees) const {
  if (!controller_ || !controllerConfig_ ||
      controllerConfig_->GetType() != MotorControllerType::Position) {
    positionDegrees = 0.0f;
    return false;
  }

  positionDegrees =
      static_cast<const Position*>(controller_)->GetTargetPositionDegrees();
  return true;
}

bool WillowMotor::GetTargetPositionPercent(float& positionPercent) const {
  if (!controller_ || !controllerConfig_ ||
      controllerConfig_->GetType() != MotorControllerType::Position) {
    positionPercent = 0.0f;
    return false;
  }

  positionPercent =
      static_cast<const Position*>(controller_)->GetTargetPositionPercent();
  return true;
}

bool WillowMotor::GetControllerOutputPercent(float& signedPercent) const {
  if (!controller_) {
    signedPercent = 0.0f;
    return false;
  }

  signedPercent = controller_->GetSignedOutputPercent();
  return true;
}

bool WillowMotor::GetDriverCurrentAmps(float& currentAmps) const {
  if (!driver_) {
    currentAmps = 0.0f;
    return false;
  }
  return driver_->GetCurrentAmps(currentAmps);
}

bool WillowMotor::GetDriverCurrentSenseRaw(uint16_t& rightRaw,
                                           uint16_t& leftRaw) const {
  if (!driver_) {
    rightRaw = 0;
    leftRaw = 0;
    return false;
  }
  return driver_->GetCurrentSenseRaw(rightRaw, leftRaw);
}

bool WillowMotor::IsDriverSoftLocked() const {
  return driver_ && driver_->IsSoftLocked();
}

bool WillowMotor::BuildComponents() {
  if (!ValidateCompatibility()) {
    return false;
  }

  if (!BuildDriver()) {
    return false;
  }
  driver_->ApplyConfig(*driverConfig_);

  if (!BuildEncoder()) {
    return false;
  }
  encoder_->ApplyConfig(*encoderConfig_);

  if (!BuildController()) {
    return false;
  }
  controller_->ApplyConfig(*controllerConfig_);

  return true;
}

bool WillowMotor::ValidateCompatibility() const {
  if (!IsDriverConfigComplete() || !IsEncoderConfigComplete() ||
      !IsControllerConfigComplete()) {
    return false;
  }

  // Position controller requires an encoder.
  if (controllerConfig_->GetType() == MotorControllerType::Position &&
      encoderConfig_->GetType() == MotorEncoderType::None) {
    return false;
  }

  return true;
}

bool WillowMotor::IsDriverConfigComplete() const {
  if (!driverConfig_) {
    return false;
  }

  if (driverConfig_->GetType() == MotorDriverType::L298N) {
    const L298NDriverConfig& cfg =
        static_cast<const L298NDriverConfig&>(*driverConfig_);
    return cfg.enaPin != 0xFF && cfg.pwmPin != 0xFF && cfg.dirPin != 0xFF;
  } else if (driverConfig_->GetType() == MotorDriverType::BTS7960) {
    const BTS7960DriverConfig& cfg =
        static_cast<const BTS7960DriverConfig&>(*driverConfig_);
    const bool basePinsValid =
        cfg.pwmLeftPin != 0xFF && cfg.pwmRightPin != 0xFF &&
        cfg.enableLeftPin != 0xFF && cfg.enableRightPin != 0xFF;
    if (!basePinsValid) {
      return false;
    }

    if (cfg.currentLimitAmps > 0.0f) {
      const bool hasSensePins = cfg.currentSenseLeftPin != 0xFF &&
                                cfg.currentSenseRightPin != 0xFF;
      return hasSensePins && cfg.adcResolutionCounts > 0 &&
             cfg.currentSenseVoltsPerAmp > 0.0f;
    }
    return true;
  }

  return false;
}

bool WillowMotor::IsEncoderConfigComplete() const {
  if (!encoderConfig_) {
    return false;
  }

  if (encoderConfig_->GetType() == MotorEncoderType::Quadrature) {
    const QuadratureEncoderConfig& cfg =
        static_cast<const QuadratureEncoderConfig&>(*encoderConfig_);
    return cfg.pinA != 0xFF && cfg.pinB != 0xFF && cfg.pulsesPerRevolution != 0;
  } else if (encoderConfig_->GetType() == MotorEncoderType::None) {
    return true;
  }

  return false;
}

bool WillowMotor::IsControllerConfigComplete() const {
  return controllerConfig_ != nullptr;
}

bool WillowMotor::BuildDriver() {
  delete driver_;
  driver_ = nullptr;

  switch (driverConfig_->GetType()) {
    case MotorDriverType::BTS7960:
      driver_ = new BTS7960();
      break;
    case MotorDriverType::L298N:
      driver_ = new L298N();
      break;
    default:
      break;
  }
  return driver_ != nullptr;
}

bool WillowMotor::BuildEncoder() {
  delete encoder_;
  encoder_ = nullptr;

  switch (encoderConfig_->GetType()) {
    case MotorEncoderType::None:
      encoder_ = new NoEncoder();
      break;
    case MotorEncoderType::Quadrature:
      encoder_ = new Quadrature();
      break;
    default:
      break;
  }
  return encoder_ != nullptr;
}

bool WillowMotor::BuildController() {
  delete controller_;
  controller_ = nullptr;

  switch (controllerConfig_->GetType()) {
    case MotorControllerType::Position:
      controller_ = new Position();
      break;
    case MotorControllerType::Speed:
      controller_ = new Speed();
      break;
    default:
      break;
  }
  return controller_ != nullptr;
}

uint32_t WillowMotor::GetConfiguredControlPeriodMicros() const {
  const ControllerConfig& config = GetControllerConfig();
  if (config.GetType() == MotorControllerType::Speed) {
    const SpeedControllerConfig& speedConfig =
        static_cast<const SpeedControllerConfig&>(config);
    return speedConfig.controlPeriodMicros;
  }

  const PositionControllerConfig& positionConfig =
      static_cast<const PositionControllerConfig&>(config);
  return positionConfig.controlPeriodMicros;
}

uint32_t WillowMotor::ReadMicros() const {
#ifdef ARDUINO
  return micros();
#else
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  return static_cast<uint32_t>(us & 0xFFFFFFFFu);
#endif
}

void WillowMotor::UpdateControllerFeedback(uint32_t deltaMicros) {
  if (!controllerConfig_ || !controller_) {
    return;
  }

  if (controllerConfig_->GetType() == MotorControllerType::Speed) {
    Speed* speedController = static_cast<Speed*>(controller_);
    speedController->SetFeedbackAvailable(IsFeedbackAvailable());
    const long pulseDelta = encoder_ ? encoder_->ConsumePulseDelta() : 0;
    speedController->SetMeasuredSpeedRpm(
        ComputeMeasuredSpeedRpm(pulseDelta, deltaMicros));
    return;
  }

  if (controllerConfig_->GetType() == MotorControllerType::Position) {
    Position* positionController = static_cast<Position*>(controller_);
    const long pulseCount = encoder_ ? encoder_->GetPulseCount() : 0;
    positionController->SetMeasuredPositionDegrees(
        ComputeMeasuredPositionDegrees(pulseCount));
  }
}

bool WillowMotor::IsFeedbackAvailable() const {
  return encoderConfig_ && encoderConfig_->GetType() != MotorEncoderType::None;
}

float WillowMotor::ComputeMeasuredSpeedRpm(long pulseDelta, uint32_t deltaMicros) const {
  if (deltaMicros == 0 || !encoderConfig_) {
    return 0.0f;
  }

  if (encoderConfig_->GetType() != MotorEncoderType::Quadrature) {
    return 0.0f;
  }

  const QuadratureEncoderConfig& quadratureConfig =
      static_cast<const QuadratureEncoderConfig&>(*encoderConfig_);
  if (quadratureConfig.pulsesPerRevolution == 0) {
    return 0.0f;
  }

  const float pulsesPerSecond =
      (static_cast<float>(pulseDelta) * 1000000.0f) / static_cast<float>(deltaMicros);
  return (pulsesPerSecond * 60.0f) /
         static_cast<float>(quadratureConfig.pulsesPerRevolution);
}

float WillowMotor::ComputeMeasuredPositionDegrees(long pulseCount) const {
  if (!encoderConfig_ || !controllerConfig_) {
    return 0.0f;
  }
  if (encoderConfig_->GetType() != MotorEncoderType::Quadrature) {
    return 0.0f;
  }
  if (controllerConfig_->GetType() != MotorControllerType::Position) {
    return 0.0f;
  }

  const QuadratureEncoderConfig& quadratureConfig =
      static_cast<const QuadratureEncoderConfig&>(*encoderConfig_);
  const PositionControllerConfig& positionConfig =
      static_cast<const PositionControllerConfig&>(*controllerConfig_);
  if (quadratureConfig.pulsesPerRevolution == 0) {
    return 0.0f;
  }

  return (static_cast<float>(pulseCount) /
          static_cast<float>(quadratureConfig.pulsesPerRevolution)) *
         positionConfig.positionReferenceDegrees;
}

bool WillowMotor::ShouldInterruptOnPhaseA() const {
  if (!encoderConfig_ || encoderConfig_->GetType() != MotorEncoderType::Quadrature) {
    return false;
  }

  const QuadratureEncoderConfig& quadratureConfig =
      static_cast<const QuadratureEncoderConfig&>(*encoderConfig_);
  return quadratureConfig.interruptMode == QuadratureInterruptMode::PhaseA ||
         quadratureConfig.interruptMode == QuadratureInterruptMode::PhaseAB;
}

bool WillowMotor::ShouldInterruptOnPhaseB() const {
  if (!encoderConfig_ || encoderConfig_->GetType() != MotorEncoderType::Quadrature) {
    return false;
  }

  const QuadratureEncoderConfig& quadratureConfig =
      static_cast<const QuadratureEncoderConfig&>(*encoderConfig_);
  return quadratureConfig.interruptMode == QuadratureInterruptMode::PhaseB ||
         quadratureConfig.interruptMode == QuadratureInterruptMode::PhaseAB;
}

void WillowMotor::QueueInterruptControlUpdate() {
  const uint8_t criticalState = EnterCritical();
  if (pendingInterruptControlUpdates_ < 0xFF) {
    ++pendingInterruptControlUpdates_;
  }
  ExitCritical(criticalState);
}

uint8_t WillowMotor::ConsumeInterruptControlUpdates() {
  const uint8_t criticalState = EnterCritical();
  const uint8_t pending = pendingInterruptControlUpdates_;
  pendingInterruptControlUpdates_ = 0;
  ExitCritical(criticalState);
  return pending;
}

uint8_t WillowMotor::GetMaxInterruptUpdatesPerLoop() const {
  if (!controllerConfig_) {
    return 1;
  }

  if (controllerConfig_->GetType() == MotorControllerType::Speed) {
    const SpeedControllerConfig& speedConfig =
        static_cast<const SpeedControllerConfig&>(*controllerConfig_);
    return speedConfig.maxInterruptUpdatesPerLoop > 0
               ? speedConfig.maxInterruptUpdatesPerLoop
               : 1;
  }

  if (controllerConfig_->GetType() == MotorControllerType::Position) {
    const PositionControllerConfig& positionConfig =
        static_cast<const PositionControllerConfig&>(*controllerConfig_);
    return positionConfig.maxInterruptUpdatesPerLoop > 0
               ? positionConfig.maxInterruptUpdatesPerLoop
               : 1;
  }

  return 1;
}

}  // namespace Willow
