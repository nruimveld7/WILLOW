# API Overview

This document summarizes the user-facing WILLOW API shipped for `1.0.0`.

## Supported Components

- `BTS7960` and `L298N` drivers
- `Quadrature` and `None` encoder modes
- `Speed` and `Position` controllers

## Configuration Model

- Driver, encoder, and controller selections are expressed through concrete
  config types in `src/common/Configs.h`
- `WillowMotor` is configured through explicit setters
- Config getters return base types so active resolved configuration can be
  inspected after `Begin()` or `ApplyConfig()`

## Control Flow

- Call `Begin()` after setting configs
- Call `Update()` from `loop()` for regular control updates
- Optionally forward encoder interrupts with `OnEncoderPhaseAChange()` and
  `OnEncoderPhaseBChange()`
- Stop output with `Stop()`

## Telemetry

- `GetDriverCurrentAmps()` exposes experimental BTS7960 current telemetry
- `GetDriverCurrentSenseRaw()` exposes raw right/left current-sense ADC values
  for users who want to characterize their own module behavior

## Command API

- Speed controllers accept `SetSpeedRpm()` and `SetSpeedPercent()`
- Position controllers accept `SetPositionDegrees()` and `SetPositionPercent()`
- Percent speed commands clamp to `[-100, +100]`
- Percent position commands normalize into `[-100, +100]`

## Example Sketches

- `examples/ClosedLoopSpeed_BTS7960_Quadrature/ClosedLoopSpeed_BTS7960_Quadrature.ino`
- `examples/OpenLoopSpeed_L298N_NoEncoder/OpenLoopSpeed_L298N_NoEncoder.ino`
- `examples/Position_BTS7960_Quadrature/Position_BTS7960_Quadrature.ino`
- `examples/Mega2560_BTS7960_Quadrature_SerialTest/Mega2560_BTS7960_Quadrature_SerialTest.ino`

Use ALDER to open, compile, and upload these sketches against the local WILLOW
library checkout.
