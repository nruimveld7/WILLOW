# WILLOW

Wrapped Interfaces for Low-level Locomotion and Odometry Workflows.

WILLOW is an Arduino motor-control library built around interchangeable motor
drivers, encoder sources, and control strategies. Version `1.0.0` ships a
stable top-level `WillowMotor` API with concrete support for:

- `BTS7960` and `L298N` motor drivers
- `Quadrature` and `None` encoder modes
- `Speed` and `Position` controllers

## Highlights

- Separation between hardware output, feedback capture, and control logic
- Closed-loop speed and position control with configurable PID gains
- Open-loop speed fallback when no encoder is present
- Runtime configuration updates through `ApplyConfig()`
- Optional interrupt-forwarded encoder/control updates
- Raw current-sense access for BTS7960-class drivers
- Single include for library consumers: `#include <WILLOW.h>`

## Installation

Install WILLOW through the Arduino Library Manager once the `1.0.0` release is
indexed, or install the repository as a ZIP in the Arduino IDE.

## Build And Test Workflow

WILLOW uses ALDER as the intended local build and upload workflow for
development and hardware testing.

1. Launch ALDER with:

```powershell
./scripts/fetch-and-run-alder.ps1
```

2. In the ALDER UI, open the sketch you want to build from `examples/`.
3. In the ALDER UI, point the project at this local WILLOW repository as the
   library source.
4. Select the target board and serial port in ALDER, then compile or upload
   from the ALDER UI.

This repository does not provide a supported headless build wrapper. ALDER is
the supported path for sketch compilation and upload during local testing.

## Quick Start

```cpp
#include <WILLOW.h>

Willow::BTS7960DriverConfig driverConfig;
Willow::QuadratureEncoderConfig encoderConfig;
Willow::SpeedControllerConfig controllerConfig;
Willow::WillowMotor motor;

void setup() {
  driverConfig.pwmLeftPin = 5;
  driverConfig.pwmRightPin = 6;
  driverConfig.enableLeftPin = 7;
  driverConfig.enableRightPin = 8;

  encoderConfig.pinA = 2;
  encoderConfig.pinB = 3;
  encoderConfig.pulsesPerRevolution = 20;

  controllerConfig.kp = 0.8f;
  controllerConfig.ki = 0.1f;
  controllerConfig.kd = 0.01f;
  controllerConfig.maxSpeedRpm = 300.0f;
  controllerConfig.speedReferenceRpm = 300.0f;

  motor.SetDriverConfig(driverConfig);
  motor.SetEncoderConfig(encoderConfig);
  motor.SetControllerConfig(controllerConfig);
  motor.Begin();
  motor.SetSpeedPercent(40.0f);
}

void loop() {
  motor.Update();
}
```

## Shipped Examples

- `examples/ClosedLoopSpeed_BTS7960_Quadrature`
- `examples/OpenLoopSpeed_L298N_NoEncoder`
- `examples/Position_BTS7960_Quadrature`
- `examples/Mega2560_BTS7960_Quadrature_SerialTest`

The Mega serial test sketch is intended for interactive hardware bring-up
through ALDER. It exposes a `Serial` command console for open-loop speed,
closed-loop speed, and closed-loop position testing.

## API Overview

The public API centers on three abstract roles:

- `MotorDriver` for hardware output
- `MotorEncoder` for feedback capture
- `MotorController` for behavior and targets

`WillowMotor` owns the active components and provides the user-facing workflow:

- Configure with `SetDriverConfig()`, `SetEncoderConfig()`, and `SetControllerConfig()`
- Start with `Begin()`
- Drive commands with `SetSpeedRpm()`, `SetSpeedPercent()`,
  `SetPositionDegrees()`, and `SetPositionPercent()`
- Run control updates with `Update()` or ISR-forwarded encoder methods
- Read experimental BTS7960 current telemetry with `GetDriverCurrentAmps()` or
  raw current-sense ADC values with `GetDriverCurrentSenseRaw()`
- Stop safely with `Stop()`

Current-sense amperes for BTS7960 modules should be treated as experimental in
`1.0.0`; raw current-sense ADC access is exposed so users can characterize
their own module behavior.

Additional design and release notes live in `docs/architecture.md`,
`docs/api-overview.md`, `docs/alder-workflow.md`, and
`docs/release-checklist.md`.
