/*
  Closed-loop speed control
  Hardware: BTS7960 + Quadrature encoder
*/

#include <WILLOW.h>

Willow::BTS7960DriverConfig driverConfig;
Willow::QuadratureEncoderConfig encoderConfig;
Willow::SpeedControllerConfig controllerConfig;

Willow::WillowMotor motor;
volatile bool g_overCurrentTrip = false;
volatile float g_lastCurrentAmps = 0.0f;

void onOverCurrent(float currentAmps) {
  g_lastCurrentAmps = currentAmps;
  g_overCurrentTrip = true;
}

void onEncoderPhaseAChange() {
  // Interrupt-driven path via phase A edges.
  motor.OnEncoderPhaseAChange();
}

void setup() {
  // BTS7960 pin model: left/right channels (not A/B).
  driverConfig.pwmLeftPin = 5;
  driverConfig.pwmRightPin = 6;
  driverConfig.currentSenseLeftPin = A0;
  driverConfig.currentSenseRightPin = A1;
  driverConfig.enableLeftPin = 7;
  driverConfig.enableRightPin = 8;
  driverConfig.currentLimitAmps = 8.0f;
  driverConfig.currentLimitLockoutMillis = 300;
  driverConfig.softLockOnOverCurrent = true;
  driverConfig.onOverCurrent = onOverCurrent;

  encoderConfig.pinA = 2;
  encoderConfig.pinB = 3;
  encoderConfig.pulsesPerRevolution = 20;
  encoderConfig.interruptMode = Willow::QuadratureInterruptMode::PhaseA;

  controllerConfig.kp = 0.8f;
  controllerConfig.ki = 0.1f;
  controllerConfig.kd = 0.01f;
  controllerConfig.maxSpeedRpm = 300.0f;
  controllerConfig.speedReferenceRpm = 300.0f;
  controllerConfig.controlPeriodMicros = 10000;  // 10 ms control step.

  motor.SetDriverConfig(driverConfig);
  motor.SetEncoderConfig(encoderConfig);
  motor.SetControllerConfig(controllerConfig);
  motor.Begin();

  // Optional: fetch effective configs after Begin/validation.
  // Generic getters return base config interfaces; concrete access is guarded
  // by type checks before downcasting.
  const Willow::DriverConfig& activeDriverConfig = motor.GetDriverConfig();
  if (activeDriverConfig.GetType() == Willow::MotorDriverType::BTS7960) {
    driverConfig = static_cast<const Willow::BTS7960DriverConfig&>(activeDriverConfig);
  }

  const Willow::EncoderConfig& activeEncoderConfig = motor.GetEncoderConfig();
  if (activeEncoderConfig.GetType() == Willow::MotorEncoderType::Quadrature) {
    encoderConfig = static_cast<const Willow::QuadratureEncoderConfig&>(activeEncoderConfig);
  }

  const Willow::ControllerConfig& activeControllerConfig = motor.GetControllerConfig();
  if (activeControllerConfig.GetType() == Willow::MotorControllerType::Speed) {
    controllerConfig = static_cast<const Willow::SpeedControllerConfig&>(activeControllerConfig);
  }

  motor.SetSpeedRpm(120.0f);
  motor.SetSpeedPercent(40.0f);  // 40% of speedReferenceRpm.

  // Direction is handled by left/right drive command mapping inside the
  // BTS7960 driver implementation.

  // User-owned interrupt wiring, when desired (phase A only in this example).
  // attachInterrupt(
  //  digitalPinToInterrupt(encoderConfig.pinA),
  //  onEncoderPhaseAChange,
  //  CHANGE
  //);
}

void loop() {
  static bool hasRetuned = false;

  // Manual update path is always supported.
  motor.Update();

  // Example over-current reaction path.
  if (g_overCurrentTrip) {
    g_overCurrentTrip = false;
    // User could log current draw and choose to stop/reduce command.
    // Serial.print("Over-current: ");
    // Serial.println(g_lastCurrentAmps);
  }

  // Runtime retune example.
  if (!hasRetuned && millis() > 2500UL) {
    controllerConfig.kp = 0.9f;
    motor.SetControllerConfig(controllerConfig);
    motor.ApplyConfig();
    motor.SetSpeedPercent(130.0f);  // Clamped to +100%.
    hasRetuned = true;
  }

  if (millis() > 5000UL) {
    motor.Stop();
  }
}
