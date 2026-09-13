# Implantable Pump Firmware

Prototype firmware for a stepper-driven syringe pump, built on ESP32 with
PlatformIO.

## Setup

1. Install the [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
   in VS Code (Extensions panel → search "PlatformIO IDE").
2. Open this folder in VS Code (**File → Open Folder**, select
   `implantable-pump-firmware/`). PlatformIO will detect `platformio.ini`
   automatically and install the ESP32 toolchain on first open — this can
   take a few minutes.
3. Plug in your ESP32 DevKit via USB.
4. Build: click the checkmark icon in the PlatformIO toolbar at the bottom
   of VS Code, or run `pio run` in the terminal.
5. Upload: click the right-arrow icon, or run `pio run -t upload`.
6. Open the Serial Monitor (plug icon, or `pio device monitor`) to see
   boot messages and send test flow-rate setpoints (type a number like
   `5.0` and hit enter).

## Project layout

```
include/config.h          Pin assignments + tunable constants (edit this for your wiring)
src/drivers/               Hardware-facing code (stepper pulse generation)
src/control/                Pure control-logic math (flow rate -> step frequency)
src/main.cpp                 FreeRTOS task wiring
test/                       Native unit tests for control logic (no hardware needed)
```

## Running tests without hardware

The flow-rate math in `src/control/` has no Arduino/ESP32 dependency, so it
can be tested on your laptop:

```
pio test -e native
```

## Switching boards

Defined environments in `platformio.ini`:
- `esp32dev` — classic ESP32 DevKit (default)
- `esp32s3` — ESP32-S3 DevKitC (ready for later)

Build/upload to a specific one with `pio run -e esp32s3 -t upload`.

## Status

Early scaffold — open-loop stepper control only, placeholder pump geometry
constants, placeholder serial command interface. See `CLAUDE.md` for
architecture notes and current TODOs.
