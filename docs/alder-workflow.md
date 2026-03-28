# ALDER Workflow

WILLOW uses ALDER as the intended developer workflow for local compile and
upload operations.

## Launch

Run:

```powershell
./scripts/fetch-and-run-alder.ps1
```

The script downloads the requested ALDER release if needed and launches the
native ALDER application.

## In ALDER

1. Open a sketch from `examples/`.
2. Add or select this local WILLOW repository as the library source.
3. Choose the target board and serial port.
4. Build or upload from the ALDER UI.

## Mega2560 Hardware Bring-Up

For the Mega + BTS7960 + quadrature encoder setup, use:

- `examples/Mega2560_BTS7960_Quadrature_SerialTest/Mega2560_BTS7960_Quadrature_SerialTest.ino`

The sketch provides a `Serial` console on UART0 for:

- open-loop speed testing
- closed-loop speed testing
- closed-loop position testing
- runtime PID and encoder-PPR adjustment

Closed-loop scaling depends on the correct encoder pulses-per-revolution value.
Set that before tuning.
