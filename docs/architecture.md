# Architecture

This document captures the core structure of the WILLOW library.

## Core Public API Bases

The design centers on three base classes that define the public API surface:

- `MotorDriver`
- `MotorEncoder`
- `MotorController`

All hardware-specific and strategy-specific behavior should be expressed through
derived classes from these bases.

## Version 1 Components

- `BTS7960` : `MotorDriver`
- `L298N` : `MotorDriver`
- `Quadrature` : `MotorEncoder`
- `Position` : `MotorController`
- `Speed` : `MotorController`

## User-Facing Selection Types

To keep hardware and workflow selection simple for users, the library uses enum
selection types in:

- `src/common/ComponentSelection.h`

Current selection enums:

- `MotorDriverType` with `BTS7960`, `L298N`
- `MotorEncoderType` with `None`, `Quadrature`
- `MotorControllerType` with `Position`, `Speed`

Selection container:

- `MotorComponentSelection` bundles driver, encoder, and controller selections.

## Config Model

- Every concrete subclass defines its own config type.
- Example: `BTS7960DriverConfig` may expose fields that
  `L298NDriverConfig` does not.
- Encoder and controller subclasses follow the same pattern.
- Command reference and normalization settings (for example speed/position
  percent reference, clamp/modulo behavior) live in controller configs.
- `WillowMotor` is default-constructed and configured through explicit setters:
  - `SetDriverConfig(...)`
  - `SetEncoderConfig(...)`
  - `SetControllerConfig(...)`
- `WillowMotor` also provides config getters so users can inspect effective
  active settings when needed:
  - `GetDriverConfig()`, `GetEncoderConfig()`, `GetControllerConfig()`
  - each returns a base config interface
    (`DriverConfig`, `EncoderConfig`, `ControllerConfig`)
  - concrete access is done by checking `GetType()` then downcasting
- `Begin()` performs compatibility checks before activation.
- Runtime config updates are supported through setters plus `ApplyConfig()`.
- Control timestep is configured in controller config through
  `controlPeriodMicros`.
- Base-class API behavior stays consistent even when optional features differ
  by hardware selection.

## API Behavior

- `WillowMotor` exposes lifecycle methods including `Begin()` and `Stop()`.
- When using `Speed` controller, user requests speed through
  `SetSpeedRpm(...)` or `SetSpeedPercent(...)`.
- Speed percent commands support signed values.
- Speed percent is clamped to `[-100, +100]` of configured speed reference/max.
- If encoder feedback is present, speed control runs closed-loop PID.
- Closed-loop implementations support either:
  - manual `Update()` calls
  - interrupt-driven updates via controller-provided interrupt methods
- `Update()` delta is internally computed by `WillowMotor`; user does not pass
  explicit delta values.
- User is responsible for wiring hardware interrupts and calling those
  controller/motor interrupt methods from their ISR handlers.
- If encoder feedback is absent, speed request remains supported through an
  open-loop fallback path.
- When using `Position` controller, an encoder is required.
- Position commands support signed degree and percent references.
- Position percent is modulo-normalized to `[-100, +100]` of `+/-360 deg`.
- `Speed` and `Position` controllers both use PID control.
- User provides PID gains; library may also provide sane default values.

## Responsibility Boundaries

- `src/interfaces/motor/`
  - Stable abstract contracts for `MotorDriver`.
  - No hardware-specific assumptions in interfaces.

- `src/interfaces/encoder/`
  - Stable abstract contracts for `MotorEncoder`.
  - No concrete signal decoding details in interfaces.

- `src/interfaces/controller/`
  - Stable abstract contracts for `MotorController`.
  - No strategy-specific implementation details in interfaces.

- `src/drivers/`
  - Concrete motor driver implementations.
  - Responsible for translating interface commands into hardware output.

- `src/encoders/`
  - Concrete encoder implementations.
  - Responsible for reading and reporting feedback through encoder interfaces.

- `src/controllers/`
  - High-level workflows and control logic derived from `MotorController`.
  - May run open-loop workflows or feedback-driven workflows depending on available encoder.

- `src/common/`
  - Shared types/utilities used across interfaces and controllers.

## Dependency Direction

Preferred dependency flow:

- Controllers depend on interfaces.
- Concrete drivers/encoders implement interfaces.
- Interfaces do not depend on concrete hardware classes.
- Higher-level logic should not require direct hardware access.

## Example Coverage

The repository includes versioned examples that exercise the primary supported
combinations:

- `examples/ClosedLoopSpeed_BTS7960_Quadrature/ClosedLoopSpeed_BTS7960_Quadrature.ino`
- `examples/OpenLoopSpeed_L298N_NoEncoder/OpenLoopSpeed_L298N_NoEncoder.ino`
- `examples/Position_BTS7960_Quadrature/Position_BTS7960_Quadrature.ino`
- `examples/Mega2560_BTS7960_Quadrature_SerialTest/Mega2560_BTS7960_Quadrature_SerialTest.ino`
