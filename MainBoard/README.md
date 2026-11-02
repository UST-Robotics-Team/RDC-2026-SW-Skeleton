# MainBoard — RDC 2026 Skeleton (STM32F405)

Skeleton firmware for the RDC 2026 MainBoard (STM32F405).

> [!WARNING]
> If ELCP motors are connected, disconnect motor power or keep the wheels
> off the ground while flashing — GPIO lines float briefly during reset.

## Quick start

Everything in this repo has been covered in tutorials with the exception of ELCP motors which are documented in the motor library directory.
Full motor API, examples and calibration notes: [Core/Inc/elcp_motor/elcp.md](Core/Inc/elcp_motor/elcp.md)
