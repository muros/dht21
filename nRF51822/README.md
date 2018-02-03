# Digital Humidity Temperature BLE Sensor

## Project IDE
Project is edited in eclipse IDE Neon.3 - CDE

Project is imported as Makefile project and is ARM Crosscompiled C project.

Set project Properties:
C/C++ Build - GNU Toolchain Editor - Autotool Makefile Generator

## Building

Makefile.common includes location of:

- GNU GCC tools for arm - (none-eabi)
- Nordic SDK (version 11)
- OpenOCD for debuging

Add Build Targets to project:
- all
- clean
- flash
- flash_erase
- flash_reset
- flash_s130

Targets can be found in Makefile and Makefile.common