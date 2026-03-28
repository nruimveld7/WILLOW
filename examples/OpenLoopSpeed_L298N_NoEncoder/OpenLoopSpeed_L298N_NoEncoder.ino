/*
  Open-loop speed request
  Hardware: L298N + No encoder
*/

#include <WILLOW.h>

Willow::L298NDriverConfig driverConfig;
Willow::NoEncoderEncoderConfig encoderConfig;
Willow::SpeedControllerConfig controllerConfig;

Willow::WillowMotor motor;

void setup() {
  // L298N pin model: Ena, Pwm, Dir.
  driverConfig.enaPin = 9;
  driverConfig.pwmPin = 10;
  driverConfig.dirPin = 11;

  // Speed controller still accepts PID gains; implementation may use a
  // simpler fallback path when feedback is unavailable.
  controllerConfig.kp = 0.8f;
  controllerConfig.ki = 0.1f;
  controllerConfig.kd = 0.01f;
  controllerConfig.maxSpeedRpm = 240.0f;
  controllerConfig.speedReferenceRpm = 240.0f;
  controllerConfig.controlPeriodMicros = 10000;  // 10 ms control step.

  motor.SetDriverConfig(driverConfig);
  motor.SetEncoderConfig(encoderConfig);
  motor.SetControllerConfig(controllerConfig);
  motor.Begin();

  // Optional: fetch effective configs after Begin/validation.
  const Willow::DriverConfig& activeDriverConfig = motor.GetDriverConfig();
  if (activeDriverConfig.GetType() == Willow::MotorDriverType::L298N) {
    driverConfig = static_cast<const Willow::L298NDriverConfig&>(activeDriverConfig);
  }

  const Willow::EncoderConfig& activeEncoderConfig = motor.GetEncoderConfig();
  if (activeEncoderConfig.GetType() == Willow::MotorEncoderType::None) {
    encoderConfig = static_cast<const Willow::NoEncoderEncoderConfig&>(activeEncoderConfig);
  }

  const Willow::ControllerConfig& activeControllerConfig = motor.GetControllerConfig();
  if (activeControllerConfig.GetType() == Willow::MotorControllerType::Speed) {
    controllerConfig = static_cast<const Willow::SpeedControllerConfig&>(activeControllerConfig);
  }

  motor.SetSpeedRpm(90.0f);
  motor.SetSpeedPercent(-50.0f);  // -50% of speedReferenceRpm.
}

void loop() {
  motor.Update();

  if (millis() > 5000UL) {
    motor.Stop();
  }
}
