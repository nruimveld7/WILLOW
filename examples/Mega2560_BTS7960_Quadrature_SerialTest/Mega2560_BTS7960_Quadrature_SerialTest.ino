/*
  Mega2560 + BTS7960 + Quadrature encoder serial test console.

  Wiring for this sketch:
  - BTS7960 RPWM -> D6
  - BTS7960 LPWM -> D7
  - BTS7960 R_EN -> D21
  - BTS7960 L_EN -> D20
  - BTS7960 R_IS -> A4
  - BTS7960 L_IS -> A8
  - Encoder A -> D18
  - Encoder B -> D19

  Serial commands:
  - help
  - mode open
  - mode speed
  - mode position
  - speedpct <percent>
  - speedrpm <rpm>
  - posdeg <degrees>
  - pospct <percent>
  - stop
  - status
  - stream on|off
  - ppr <pulses_per_revolution>
  - pid <kp> <ki> <kd>
  - speeddeadband <rpm>
  - posdeadband <degrees>
  - currentscale <volts_per_amp>
  - currentraw
  - maxrpm <rpm>
  - refrpm <rpm>
  - posref <degrees>
  - period <micros>
  - pullups on|off
  - interrupt manual|a|b|ab
*/

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include <WILLOW.h>

namespace {

constexpr uint8_t kBtsRpmwPin = 6;
constexpr uint8_t kBtsLpwmPin = 7;
constexpr uint8_t kBtsRightEnablePin = 21;
constexpr uint8_t kBtsLeftEnablePin = 20;
constexpr uint8_t kBtsRightCurrentSensePin = A4;
constexpr uint8_t kBtsLeftCurrentSensePin = A8;
constexpr uint8_t kEncoderPhaseAPin = 18;
constexpr uint8_t kEncoderPhaseBPin = 19;
constexpr float kAdcReferenceVolts = 5.0f;
constexpr uint16_t kAdcResolutionCounts = 1023;

constexpr unsigned long kStatusIntervalMs = 1000UL;
constexpr size_t kLineBufferSize = 96;
constexpr uint16_t kDefaultPulsesPerRevolution = 1920;

enum class TestMode {
  OpenLoopSpeed,
  ClosedLoopSpeed,
  ClosedLoopPosition
};

Willow::WillowMotor g_motor;
Willow::BTS7960DriverConfig g_driverConfig;
Willow::QuadratureEncoderConfig g_quadratureConfig;
Willow::NoEncoderEncoderConfig g_noEncoderConfig;
Willow::SpeedControllerConfig g_speedConfig;
Willow::PositionControllerConfig g_positionConfig;

TestMode g_mode = TestMode::OpenLoopSpeed;
char g_lineBuffer[kLineBufferSize];
size_t g_lineLength = 0;
unsigned long g_lastStatusMs = 0;
bool g_motorStarted = false;
bool g_streamStatus = false;
volatile long g_monitorPulseCount = 0;
volatile uint8_t g_monitorLastState = 0;
volatile bool g_monitorHasState = false;
long g_lastStatusPulseCount = 0;
unsigned long g_lastStatusSampleMs = 0;

uint8_t ReadEncoderState() {
  const uint8_t a = digitalRead(kEncoderPhaseAPin) ? 1u : 0u;
  const uint8_t b = digitalRead(kEncoderPhaseBPin) ? 1u : 0u;
  return static_cast<uint8_t>((a << 1) | b);
}

int8_t DecodeQuadratureTransition(uint8_t previous, uint8_t next) {
  static const int8_t kTransitionTable[16] = {
      0,  -1, 1,  0,   // 00 -> 00/01/10/11
      1,  0,  0,  -1,  // 01 -> 00/01/10/11
      -1, 0,  0,  1,   // 10 -> 00/01/10/11
      0,  1,  -1, 0    // 11 -> 00/01/10/11
  };
  return kTransitionTable[((previous & 0x3u) << 2) | (next & 0x3u)];
}

void UpdateEncoderMonitor() {
  const uint8_t nextState = ReadEncoderState();
  if (!g_monitorHasState) {
    g_monitorLastState = nextState;
    g_monitorHasState = true;
    return;
  }

  g_monitorPulseCount += DecodeQuadratureTransition(g_monitorLastState, nextState);
  g_monitorLastState = nextState;
}

long GetMonitorPulseCount() {
  noInterrupts();
  const long pulseCount = g_monitorPulseCount;
  interrupts();
  return pulseCount;
}

void ResetEncoderMonitor() {
  noInterrupts();
  g_monitorPulseCount = 0;
  g_monitorLastState = ReadEncoderState();
  g_monitorHasState = true;
  interrupts();
  g_lastStatusPulseCount = 0;
  g_lastStatusSampleMs = millis();
}

const char* ModeName(TestMode mode) {
  switch (mode) {
    case TestMode::OpenLoopSpeed:
      return "open";
    case TestMode::ClosedLoopSpeed:
      return "speed";
    case TestMode::ClosedLoopPosition:
      return "position";
  }
  return "unknown";
}

const char* InterruptModeName(Willow::QuadratureInterruptMode mode) {
  switch (mode) {
    case Willow::QuadratureInterruptMode::Manual:
      return "manual";
    case Willow::QuadratureInterruptMode::PhaseA:
      return "a";
    case Willow::QuadratureInterruptMode::PhaseB:
      return "b";
    case Willow::QuadratureInterruptMode::PhaseAB:
      return "ab";
  }
  return "unknown";
}

void OnEncoderPhaseAChange() {
  UpdateEncoderMonitor();
  if (g_mode != TestMode::OpenLoopSpeed && g_motorStarted &&
      g_quadratureConfig.interruptMode != Willow::QuadratureInterruptMode::Manual) {
    g_motor.OnEncoderPhaseAChange();
  }
}

void OnEncoderPhaseBChange() {
  UpdateEncoderMonitor();
  if (g_mode != TestMode::OpenLoopSpeed && g_motorStarted &&
      g_quadratureConfig.interruptMode != Willow::QuadratureInterruptMode::Manual) {
    g_motor.OnEncoderPhaseBChange();
  }
}

void PrintPrompt() { Serial.print(F("> ")); }

void PrintHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  mode open|speed|position"));
  Serial.println(F("  speedpct <percent>"));
  Serial.println(F("  speedrpm <rpm>"));
  Serial.println(F("  posdeg <degrees>"));
  Serial.println(F("  pospct <percent>"));
  Serial.println(F("  stop"));
  Serial.println(F("  status"));
  Serial.println(F("  stream on|off"));
  Serial.println(F("  ppr <pulses_per_revolution>"));
  Serial.println(F("  pid <kp> <ki> <kd>"));
  Serial.println(F("  speeddeadband <rpm>"));
  Serial.println(F("  posdeadband <degrees>"));
  Serial.println(F("  currentscale <volts_per_amp>"));
  Serial.println(F("  currentraw"));
  Serial.println(F("  maxrpm <rpm>"));
  Serial.println(F("  refrpm <rpm>"));
  Serial.println(F("  posref <degrees>"));
  Serial.println(F("  period <micros>"));
  Serial.println(F("  pullups on|off"));
  Serial.println(F("  interrupt manual|a|b|ab"));
  Serial.println(F("Closed-loop scaling depends on the correct encoder PPR."));
}

void AttachEncoderInterrupts() {
  detachInterrupt(digitalPinToInterrupt(kEncoderPhaseAPin));
  detachInterrupt(digitalPinToInterrupt(kEncoderPhaseBPin));
  attachInterrupt(digitalPinToInterrupt(kEncoderPhaseAPin), OnEncoderPhaseAChange,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderPhaseBPin), OnEncoderPhaseBChange,
                  CHANGE);
}

bool ApplyModeConfig(TestMode mode) {
  g_motor.Stop();
  g_motorStarted = false;
  detachInterrupt(digitalPinToInterrupt(kEncoderPhaseAPin));
  detachInterrupt(digitalPinToInterrupt(kEncoderPhaseBPin));

  g_mode = mode;
  g_motor.SetDriverConfig(g_driverConfig);
  if (mode == TestMode::OpenLoopSpeed) {
    g_motor.SetEncoderConfig(g_noEncoderConfig);
    g_motor.SetControllerConfig(g_speedConfig);
  } else if (mode == TestMode::ClosedLoopSpeed) {
    g_motor.SetEncoderConfig(g_quadratureConfig);
    g_motor.SetControllerConfig(g_speedConfig);
  } else {
    g_motor.SetEncoderConfig(g_quadratureConfig);
    g_motor.SetControllerConfig(g_positionConfig);
  }

  g_motorStarted = g_motor.Begin();
  ResetEncoderMonitor();
  AttachEncoderInterrupts();
  return g_motorStarted;
}

void PrintStatus() {
  float outputPercent = 0.0f;
  float measuredSpeedRpm = 0.0f;
  float measuredPositionDegrees = 0.0f;
  float targetSpeedRpm = 0.0f;
  float targetSpeedPercent = 0.0f;
  float targetPositionDegrees = 0.0f;
  float targetPositionPercent = 0.0f;
  float currentAmps = 0.0f;
  const bool hasOutput = g_motor.GetControllerOutputPercent(outputPercent);
  bool hasMeasuredSpeed = g_motor.GetMeasuredSpeedRpm(measuredSpeedRpm);
  const bool hasMeasuredPosition =
      g_motor.GetMeasuredPositionDegrees(measuredPositionDegrees);
  const bool hasTargetSpeedRpm = g_motor.GetTargetSpeedRpm(targetSpeedRpm);
  const bool hasTargetSpeedPercent =
      g_motor.GetTargetSpeedPercent(targetSpeedPercent);
  const bool hasTargetPositionDegrees =
      g_motor.GetTargetPositionDegrees(targetPositionDegrees);
  const bool hasTargetPositionPercent =
      g_motor.GetTargetPositionPercent(targetPositionPercent);
  const bool hasCurrent = g_motor.GetDriverCurrentAmps(currentAmps);
  const int rawCurrentRight = analogRead(kBtsRightCurrentSensePin);
  const int rawCurrentLeft = analogRead(kBtsLeftCurrentSensePin);
  const float rightCurrentVolts =
      (static_cast<float>(rawCurrentRight) * kAdcReferenceVolts) /
      static_cast<float>(kAdcResolutionCounts);
  const float leftCurrentVolts =
      (static_cast<float>(rawCurrentLeft) * kAdcReferenceVolts) /
      static_cast<float>(kAdcResolutionCounts);
  const long pulseCount = GetMonitorPulseCount();
  const unsigned long now = millis();
  if (g_mode == TestMode::OpenLoopSpeed) {
    const unsigned long deltaMs = now - g_lastStatusSampleMs;
    const long pulseDelta = pulseCount - g_lastStatusPulseCount;
    if (deltaMs > 0 && g_quadratureConfig.pulsesPerRevolution > 0) {
      measuredSpeedRpm =
          (static_cast<float>(pulseDelta) * 60000.0f) /
          (static_cast<float>(g_quadratureConfig.pulsesPerRevolution) *
           static_cast<float>(deltaMs));
      hasMeasuredSpeed = true;
    }
  }
  g_lastStatusPulseCount = pulseCount;
  g_lastStatusSampleMs = now;

  Serial.print(F("mode="));
  Serial.print(ModeName(g_mode));
  Serial.print(F(" started="));
  Serial.print(g_motorStarted ? F("yes") : F("no"));
  Serial.print(F(" ppr="));
  Serial.print(g_quadratureConfig.pulsesPerRevolution);
  Serial.print(F(" pulses="));
  Serial.print(pulseCount);
  Serial.print(F(" output_pct="));
  Serial.print(hasOutput ? outputPercent : 0.0f, 2);
  if (hasTargetSpeedPercent) {
    Serial.print(F(" target_speed_pct="));
    Serial.print(targetSpeedPercent, 2);
  }
  if (hasTargetSpeedRpm) {
    Serial.print(F(" target_speed_rpm="));
    Serial.print(targetSpeedRpm, 2);
  }
  if (hasMeasuredSpeed) {
    Serial.print(F(" speed_rpm="));
    Serial.print(measuredSpeedRpm, 2);
  }
  if (hasTargetPositionPercent) {
    Serial.print(F(" target_pos_pct="));
    Serial.print(targetPositionPercent, 2);
  }
  if (hasTargetPositionDegrees) {
    Serial.print(F(" target_pos_deg="));
    Serial.print(targetPositionDegrees, 2);
  }
  if (hasMeasuredPosition) {
    Serial.print(F(" pos_deg="));
    Serial.print(measuredPositionDegrees, 2);
  }
  Serial.print(F(" speed_db_rpm="));
  Serial.print(g_speedConfig.zeroSpeedHoldDeadbandRpm, 2);
  Serial.print(F(" pos_db_deg="));
  Serial.print(g_positionConfig.holdDeadbandDegrees, 2);
  Serial.print(F(" is_r_raw="));
  Serial.print(rawCurrentRight);
  Serial.print(F(" is_r_v="));
  Serial.print(rightCurrentVolts, 3);
  Serial.print(F(" is_l_raw="));
  Serial.print(rawCurrentLeft);
  Serial.print(F(" is_l_v="));
  Serial.print(leftCurrentVolts, 3);
  Serial.print(F(" current_scale_vpa="));
  Serial.print(g_driverConfig.currentSenseVoltsPerAmp, 4);
  if (hasCurrent) {
    Serial.print(F(" current_a="));
    Serial.print(currentAmps, 2);
  } else {
    Serial.print(F(" current_a=disabled"));
  }
  Serial.print(F(" softlock="));
  Serial.print(g_motor.IsDriverSoftLocked() ? F("yes") : F("no"));
  Serial.print(F(" irq="));
  Serial.println(InterruptModeName(g_quadratureConfig.interruptMode));
}

bool ParseFloatToken(char* token, float& value) {
  if (token == nullptr) {
    return false;
  }

  char* end = nullptr;
  value = static_cast<float>(strtod(token, &end));
  return end != token && end != nullptr && *end == '\0';
}

bool ParseUnsignedLongToken(char* token, unsigned long& value) {
  if (token == nullptr) {
    return false;
  }

  char* end = nullptr;
  value = strtoul(token, &end, 10);
  return end != token && end != nullptr && *end == '\0';
}

void PrintCommandResult(bool ok, const __FlashStringHelper* okMessage) {
  if (ok) {
    Serial.println(okMessage);
  } else {
    Serial.println(F("Command failed."));
  }
}

void HandleModeCommand(char* arg) {
  if (arg == nullptr) {
    Serial.println(F("Usage: mode open|speed|position"));
    return;
  }

  bool ok = false;
  if (strcmp(arg, "open") == 0) {
    ok = ApplyModeConfig(TestMode::OpenLoopSpeed);
  } else if (strcmp(arg, "speed") == 0) {
    ok = ApplyModeConfig(TestMode::ClosedLoopSpeed);
  } else if (strcmp(arg, "position") == 0) {
    ok = ApplyModeConfig(TestMode::ClosedLoopPosition);
  } else {
    Serial.println(F("Unknown mode."));
    return;
  }

  PrintCommandResult(ok, F("Mode applied."));
}

void HandlePidCommand(char* kpToken, char* kiToken, char* kdToken) {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  if (!ParseFloatToken(kpToken, kp) || !ParseFloatToken(kiToken, ki) ||
      !ParseFloatToken(kdToken, kd)) {
    Serial.println(F("Usage: pid <kp> <ki> <kd>"));
    return;
  }

  g_speedConfig.kp = kp;
  g_speedConfig.ki = ki;
  g_speedConfig.kd = kd;
  g_positionConfig.kp = kp;
  g_positionConfig.ki = ki;
  g_positionConfig.kd = kd;
  PrintCommandResult(ApplyModeConfig(g_mode), F("PID updated."));
}

void HandleCommand(char* line) {
  char* command = strtok(line, " ");
  if (command == nullptr) {
    return;
  }

  if (strcmp(command, "help") == 0) {
    PrintHelp();
    return;
  }

  if (strcmp(command, "mode") == 0) {
    HandleModeCommand(strtok(nullptr, " "));
    return;
  }

  if (strcmp(command, "speedpct") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value)) {
      Serial.println(F("Usage: speedpct <percent>"));
      return;
    }
    PrintCommandResult(g_motor.SetSpeedPercent(value), F("Speed percent set."));
    return;
  }

  if (strcmp(command, "speedrpm") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value)) {
      Serial.println(F("Usage: speedrpm <rpm>"));
      return;
    }
    PrintCommandResult(g_motor.SetSpeedRpm(value), F("Speed RPM set."));
    return;
  }

  if (strcmp(command, "posdeg") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value)) {
      Serial.println(F("Usage: posdeg <degrees>"));
      return;
    }
    PrintCommandResult(g_motor.SetPositionDegrees(value),
                       F("Position degrees set."));
    return;
  }

  if (strcmp(command, "pospct") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value)) {
      Serial.println(F("Usage: pospct <percent>"));
      return;
    }
    PrintCommandResult(g_motor.SetPositionPercent(value),
                       F("Position percent set."));
    return;
  }

  if (strcmp(command, "stop") == 0) {
    g_motor.Stop();
    g_motorStarted = false;
    AttachEncoderInterrupts();
    Serial.println(F("Motor stopped. Reapply a mode to continue."));
    return;
  }

  if (strcmp(command, "status") == 0) {
    PrintStatus();
    return;
  }

  if (strcmp(command, "currentraw") == 0) {
    const int rawCurrentRight = analogRead(kBtsRightCurrentSensePin);
    const int rawCurrentLeft = analogRead(kBtsLeftCurrentSensePin);
    const float rightCurrentVolts =
        (static_cast<float>(rawCurrentRight) * kAdcReferenceVolts) /
        static_cast<float>(kAdcResolutionCounts);
    const float leftCurrentVolts =
        (static_cast<float>(rawCurrentLeft) * kAdcReferenceVolts) /
        static_cast<float>(kAdcResolutionCounts);
    Serial.print(F("R_IS raw="));
    Serial.print(rawCurrentRight);
    Serial.print(F(" volts="));
    Serial.print(rightCurrentVolts, 3);
    Serial.print(F(" L_IS raw="));
    Serial.print(rawCurrentLeft);
    Serial.print(F(" volts="));
    Serial.println(leftCurrentVolts, 3);
    return;
  }

  if (strcmp(command, "ppr") == 0) {
    unsigned long value = 0;
    if (!ParseUnsignedLongToken(strtok(nullptr, " "), value) || value == 0UL ||
        value > 65535UL) {
      Serial.println(F("Usage: ppr <1..65535>"));
      return;
    }
    g_quadratureConfig.pulsesPerRevolution = static_cast<uint16_t>(value);
    PrintCommandResult(ApplyModeConfig(g_mode), F("Encoder PPR updated."));
    return;
  }

  if (strcmp(command, "stream") == 0) {
    char* arg = strtok(nullptr, " ");
    if (arg == nullptr || (strcmp(arg, "on") != 0 && strcmp(arg, "off") != 0)) {
      Serial.println(F("Usage: stream on|off"));
      return;
    }
    g_streamStatus = strcmp(arg, "on") == 0;
    Serial.println(g_streamStatus ? F("Status streaming enabled.")
                                  : F("Status streaming disabled."));
    return;
  }

  if (strcmp(command, "maxrpm") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value) || value < 0.0f) {
      Serial.println(F("Usage: maxrpm <rpm>"));
      return;
    }
    g_speedConfig.maxSpeedRpm = value;
    PrintCommandResult(ApplyModeConfig(g_mode), F("Max RPM updated."));
    return;
  }

  if (strcmp(command, "speeddeadband") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value) || value < 0.0f) {
      Serial.println(F("Usage: speeddeadband <rpm>"));
      return;
    }
    g_speedConfig.zeroSpeedHoldDeadbandRpm = value;
    PrintCommandResult(ApplyModeConfig(g_mode), F("Speed deadband updated."));
    return;
  }

  if (strcmp(command, "posdeadband") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value) || value < 0.0f) {
      Serial.println(F("Usage: posdeadband <degrees>"));
      return;
    }
    g_positionConfig.holdDeadbandDegrees = value;
    PrintCommandResult(ApplyModeConfig(g_mode), F("Position deadband updated."));
    return;
  }

  if (strcmp(command, "currentscale") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value) || value < 0.0f) {
      Serial.println(F("Usage: currentscale <volts_per_amp>"));
      return;
    }
    g_driverConfig.currentSenseVoltsPerAmp = value;
    PrintCommandResult(ApplyModeConfig(g_mode), F("Current scale updated."));
    return;
  }

  if (strcmp(command, "refrpm") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value) || value < 0.0f) {
      Serial.println(F("Usage: refrpm <rpm>"));
      return;
    }
    g_speedConfig.speedReferenceRpm = value;
    PrintCommandResult(ApplyModeConfig(g_mode), F("Reference RPM updated."));
    return;
  }

  if (strcmp(command, "posref") == 0) {
    float value = 0.0f;
    if (!ParseFloatToken(strtok(nullptr, " "), value) || value <= 0.0f) {
      Serial.println(F("Usage: posref <degrees>"));
      return;
    }
    g_positionConfig.positionReferenceDegrees = value;
    PrintCommandResult(ApplyModeConfig(g_mode),
                       F("Position reference updated."));
    return;
  }

  if (strcmp(command, "period") == 0) {
    unsigned long value = 0;
    if (!ParseUnsignedLongToken(strtok(nullptr, " "), value) || value == 0UL) {
      Serial.println(F("Usage: period <micros>"));
      return;
    }
    g_speedConfig.controlPeriodMicros = static_cast<uint32_t>(value);
    g_positionConfig.controlPeriodMicros = static_cast<uint32_t>(value);
    PrintCommandResult(ApplyModeConfig(g_mode), F("Control period updated."));
    return;
  }

  if (strcmp(command, "pullups") == 0) {
    char* arg = strtok(nullptr, " ");
    if (arg == nullptr ||
        (strcmp(arg, "on") != 0 && strcmp(arg, "off") != 0)) {
      Serial.println(F("Usage: pullups on|off"));
      return;
    }
    g_quadratureConfig.useInternalPullups = strcmp(arg, "on") == 0;
    PrintCommandResult(ApplyModeConfig(g_mode), F("Pull-up setting updated."));
    return;
  }

  if (strcmp(command, "interrupt") == 0) {
    char* arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println(F("Usage: interrupt manual|a|b|ab"));
      return;
    }

    if (strcmp(arg, "manual") == 0) {
      g_quadratureConfig.interruptMode = Willow::QuadratureInterruptMode::Manual;
    } else if (strcmp(arg, "a") == 0) {
      g_quadratureConfig.interruptMode = Willow::QuadratureInterruptMode::PhaseA;
    } else if (strcmp(arg, "b") == 0) {
      g_quadratureConfig.interruptMode = Willow::QuadratureInterruptMode::PhaseB;
    } else if (strcmp(arg, "ab") == 0) {
      g_quadratureConfig.interruptMode = Willow::QuadratureInterruptMode::PhaseAB;
    } else {
      Serial.println(F("Usage: interrupt manual|a|b|ab"));
      return;
    }

    PrintCommandResult(ApplyModeConfig(g_mode),
                       F("Interrupt mode updated."));
    return;
  }

  if (strcmp(command, "pid") == 0) {
    char* kpToken = strtok(nullptr, " ");
    char* kiToken = strtok(nullptr, " ");
    char* kdToken = strtok(nullptr, " ");
    HandlePidCommand(kpToken, kiToken, kdToken);
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
}

void ProcessSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      g_lineBuffer[g_lineLength] = '\0';
      HandleCommand(g_lineBuffer);
      g_lineLength = 0;
      PrintPrompt();
      continue;
    }

    if (g_lineLength + 1 < kLineBufferSize) {
      g_lineBuffer[g_lineLength++] = incoming;
    }
  }
}

void ApplyDefaultConfig() {
  g_driverConfig.pwmRightPin = kBtsRpmwPin;
  g_driverConfig.pwmLeftPin = kBtsLpwmPin;
  g_driverConfig.enableRightPin = kBtsRightEnablePin;
  g_driverConfig.enableLeftPin = kBtsLeftEnablePin;
  g_driverConfig.currentSenseRightPin = kBtsRightCurrentSensePin;
  g_driverConfig.currentSenseLeftPin = kBtsLeftCurrentSensePin;
  g_driverConfig.currentSenseVoltsPerAmp = 0.0f;
  g_driverConfig.currentLimitAmps = 0.0f;
  g_driverConfig.softLockOnOverCurrent = false;

  g_quadratureConfig.pinA = kEncoderPhaseAPin;
  g_quadratureConfig.pinB = kEncoderPhaseBPin;
  g_quadratureConfig.pulsesPerRevolution = kDefaultPulsesPerRevolution;
  g_quadratureConfig.useInternalPullups = false;
  g_quadratureConfig.interruptMode = Willow::QuadratureInterruptMode::PhaseAB;

  pinMode(kEncoderPhaseAPin, INPUT);
  pinMode(kEncoderPhaseBPin, INPUT);
  ResetEncoderMonitor();

  g_speedConfig.kp = 0.8f;
  g_speedConfig.ki = 0.1f;
  g_speedConfig.kd = 0.01f;
  g_speedConfig.maxSpeedRpm = 150.0f;
  g_speedConfig.speedReferenceRpm = 150.0f;
  g_speedConfig.zeroSpeedHoldDeadbandRpm = 2.0f;
  g_speedConfig.controlPeriodMicros = 10000UL;
  g_speedConfig.maxInterruptUpdatesPerLoop = 8;

  g_positionConfig.kp = 1.2f;
  g_positionConfig.ki = 0.0f;
  g_positionConfig.kd = 0.02f;
  g_positionConfig.positionReferenceDegrees = 360.0f;
  g_positionConfig.holdDeadbandDegrees = 2.0f;
  g_positionConfig.controlPeriodMicros = 10000UL;
  g_positionConfig.maxInterruptUpdatesPerLoop = 8;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  ApplyDefaultConfig();
  const bool started = ApplyModeConfig(TestMode::OpenLoopSpeed);

  Serial.println(F("WILLOW Mega2560 serial test"));
  Serial.println(F("Default mode: open-loop speed"));
  Serial.println(
      F("Current sense telemetry is disabled until currentscale is set."));
  Serial.println(F("Set the correct encoder PPR before closed-loop tuning."));
  Serial.println(started ? F("Motor initialized.") : F("Motor init failed."));
  PrintHelp();
  PrintPrompt();
}

void loop() {
  ProcessSerialInput();
  g_motor.Update();

  const unsigned long now = millis();
  if (g_streamStatus && now - g_lastStatusMs >= kStatusIntervalMs) {
    g_lastStatusMs = now;
    PrintStatus();
    PrintPrompt();
  }
}
