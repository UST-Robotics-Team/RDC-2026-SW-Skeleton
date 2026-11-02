# RDC 2026 - Software Skeleton

Starting skeletons for the UST Robotics Team RDC 2026 software tutorial. Two
independent STM32 firmware projects:

| Project                               | Board                 | Contents                                                                                                                                                   |
| ------------------------------------- | --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`MainBoard/`](MainBoard/README.md)   | STM32F405             | ELCP DC motor drivers (J7/J8), TFT status display with buttons/LEDs, and IR / ultrasonic / TOF sensor APIs. See [MainBoard/README.md](MainBoard/README.md) |
| [`Controller/`](Controller/README.md) | Blue Pill (STM32F103) | Joystick / dual-axis ADC acquisition, normalization, and calibration starter. See [Controller/README.md](Controller/README.md)                             |

Each project is self-contained with its own CubeMX `.ioc`, HAL drivers, and CMake build.
