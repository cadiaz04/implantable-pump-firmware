# Project context for AI assistants

## What this is
Firmware for a prototype implantable drug-delivery pump, built for a senior
design project (Rice BIOE). This is a *prototyping* platform — final
implantable electronics will differ, but the control algorithms and firmware
architecture developed here are meant to carry forward.

## Hardware (current bench setup)
- MCU: classic ESP32 DevKit (see `platformio.ini` — `esp32dev` env is
  default; an `esp32s3` env exists for if/when we move boards)
- Actuator: stepper motor driving a syringe pump via lead screw, through a
  generic STEP/DIR driver (A4988/DRV8825/TMC2209-style)
- Pin assignments live in `include/config.h` — that is the single source of
  truth for wiring. Update it, not scattered pin numbers elsewhere.

## Architecture
- `src/drivers/` — hardware-facing code only (e.g. `stepper_driver` talks
  directly to GPIO/LEDC). No control logic here.
- `src/control/` — pure control-logic/math, deliberately free of Arduino
  dependencies so it can be unit tested natively (`pio test -e native`).
  `flow_controller` converts a flow-rate setpoint (mL/hr) into a step
  frequency (Hz) and enforces safety bounds.
- `src/main.cpp` — wires it together. Runs the control loop as its own
  FreeRTOS task (`controlTask`, pinned to core 1) on a fixed period defined
  by `CONTROL_LOOP_PERIOD_MS`, so pump timing stays deterministic regardless
  of what `loop()` is doing (serial I/O, future BLE, etc.).

## Safety-relevant conventions (please preserve these when editing)
- `FlowController::computeStepFrequencyHz()` REJECTS out-of-range or
  NaN/inf setpoints (returns 0 Hz, `isSafe() == false`) rather than silently
  clamping them. The caller (`controlTask`) must stop the motor on
  `isSafe() == false`. Do not "helpfully" change this to auto-clamp without
  discussing it — for a device meant to deliver drug into a body, failing
  closed is the correct default.
- Control logic and hardware drivers are kept in separate files/modules on
  purpose (see Architecture above). Keep new control logic out of
  `stepper_driver.*` and new hardware calls out of `flow_controller.*` — this
  is what keeps the safety-critical math testable without hardware.
- `MAX_FLOW_RATE_ML_HR` / `MIN_FLOW_RATE_ML_HR` in `config.h` are hard
  firmware-level ceilings, independent of any higher-level "smart" logic
  added later (e.g. closed-loop sensor feedback, PID). Any new control mode
  should still be bounded by these.

## Known placeholders / TODOs
- `PumpGeometry` values in `config.h` are placeholders — need real numbers
  once the syringe/lead-screw hardware is finalized.
- `main.cpp`'s serial interface (`Serial.parseFloat()`) is a bench-test
  stand-in, not a real command protocol.
- No closed-loop feedback yet (e.g. flow sensor, pressure sensor) — current
  control is open-loop step-rate only.

## Useful commands
- Build: `pio run -e esp32dev`
- Upload + monitor: `pio run -e esp32dev -t upload -t monitor`
- Run pure-logic unit tests (no hardware needed): `pio test -e native`
