/*
  Closed-loop position control
  Hardware: BTS7960 + Quadrature encoder
*/

#include <WILLOW.h>

Willow::BTS7960DriverConfig driverConfig;
Willow::QuadratureEncoderConfig encoderConfig;
Willow::PositionControllerConfig controllerConfig;

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

void onEncoderPhaseBChange() {
  // Interrupt-driven path via phase B edges.
  motor.OnEncoderPhaseBChange();
}

void setup() {
  // BTS7960 pin model: left/right channels (not A/B).
  driverConfig.pwmLeftPin = 7;
  driverConfig.pwmRightPin = 8;
  driverConfig.currentSenseLeftPin = A2;
  driverConfig.currentSenseRightPin = A3;
  driverConfig.enableLeftPin = 9;
  driverConfig.enableRightPin = 10;
  driverConfig.currentLimitAmps = 8.0f;
  driverConfig.currentLimitLockoutMillis = 300;
  driverConfig.softLockOnOverCurrent = true;
  driverConfig.onOverCurrent = onOverCurrent;

  encoderConfig.pinA = 18;
  encoderConfig.pinB = 19;
  encoderConfig.pulsesPerRevolution = 20;
  encoderConfig.interruptMode = Willow::QuadratureInterruptMode::PhaseAB;

  controllerConfig.kp = 1.2f;
  controllerConfig.ki = 0.0f;
  controllerConfig.kd = 0.02f;
  controllerConfig.positionReferenceDegrees = 360.0f;
  controllerConfig.controlPeriodMicros = 10000;  // 10 ms control step.

  motor.SetDriverConfig(driverConfig);
  motor.SetEncoderConfig(encoderConfig);
  motor.SetControllerConfig(controllerConfig);
  motor.Begin();

  // Optional: fetch effective configs after Begin/validation.
  const Willow::DriverConfig& activeDriverConfig = motor.GetDriverConfig();
  if (activeDriverConfig.GetType() == Willow::MotorDriverType::BTS7960) {
    driverConfig = static_cast<const Willow::BTS7960DriverConfig&>(activeDriverConfig);
  }

  const Willow::EncoderConfig& activeEncoderConfig = motor.GetEncoderConfig();
  if (activeEncoderConfig.GetType() == Willow::MotorEncoderType::Quadrature) {
    encoderConfig = static_cast<const Willow::QuadratureEncoderConfig&>(activeEncoderConfig);
  }

  const Willow::ControllerConfig& activeControllerConfig = motor.GetControllerConfig();
  if (activeControllerConfig.GetType() == Willow::MotorControllerType::Position) {
    controllerConfig = static_cast<const Willow::PositionControllerConfig&>(activeControllerConfig);
  }

  motor.SetPositionDegrees(90.0f);
  motor.SetPositionPercent(25.0f);  // 25% of +360 deg reference.

  // Direction is handled by left/right drive command mapping inside the
  // BTS7960 driver implementation.

  // User-owned interrupt wiring, when desired (both phase A and phase B).
  // attachInterrupt(digitalPinToInterrupt(encoderConfig.pinA),
  //                 onEncoderPhaseAChange,
  //                 CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoderConfig.pinB),
  //                 onEncoderPhaseBChange,
  //                 CHANGE);
}

void loop() {
  static bool hasWrapped = false;

  // Manual update path is always supported.
  motor.Update();

  // Example over-current reaction path.
  if (g_overCurrentTrip) {
    g_overCurrentTrip = false;
    // User could log current draw and choose to stop/reduce command.
    // Serial.print("Over-current: ");
    // Serial.println(g_lastCurrentAmps);
  }

  if (!hasWrapped && millis() > 2500UL) {
    motor.SetPositionPercent(250.0f);  // Modulo to a value in [-100, +100].
    hasWrapped = true;
  }

  if (millis() > 5000UL) {
    motor.Stop();
  }
}
