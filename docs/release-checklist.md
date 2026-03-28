# Release Checklist

Use this checklist when publishing WILLOW `1.0.0`.

## Repository

- Confirm `library.properties` version matches the Git tag
- Confirm README examples and shipped example sketches are current
- Confirm no placeholder text remains in publish-facing files
- Confirm the public header remains `src/WILLOW.h`

## Validation

- Compile each example in ALDER against the target boards you intend to support
- Verify interrupt wiring notes match tested hardware behavior
- Smoke-test open-loop and closed-loop control on real hardware
- Treat BTS7960 amperage telemetry as experimental unless the module-specific
  current-sense path has been calibrated and filtered

## Publish

- Create and push the `1.0.0` Git tag
- Create a GitHub release for `1.0.0`
- Submit the repository URL to the Arduino Library Manager index
- Monitor the Library Manager ingestion PR until merged
