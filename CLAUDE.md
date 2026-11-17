# Project context for AI assistants

## What this is
Firmware + web UI ("BioPump Controller") for bench-testing prototype pumps
for an implantable intrathecal drug-delivery pump (Rice BIOE senior design).
This is a *prototyping* platform. The final implantable electronics will differ,
but the control algorithms and firmware architecture are meant to carry
forward. User-facing docs: README.md and docs/ (WIRING, CALIBRATION, API,
ASSUMPTIONS).

## Hardware (current bench setup)
- MCU: ESP32-WROOM-32 DevKit V1 (`esp32dev`). Platform pinned to
  espressif32 6.12.0 (Arduino core 2.0.17 / IDF 4.4); the step engine uses the
  IDF 4.4 timer-group API.
- Up to 2 pump channels. Per channel: ULN2003 (28BYJ-48), STEP/DIR
  (A4988/DRV8825/TMC2209), DRV8871 or PWM+DIR DC driver, selectable at runtime.
- Sensirion SLF3S-1300F flow sensor per channel, each on its own I²C bus
  (fixed address 0x08). Optional SparkFun Qwiic MicroPressure (Honeywell
  MPRLS0025PA00001A, absolute 0-25 psi, 0x18) shares that channel's bus.
  The channel owns the bus (`Wire`/`Wire1` begin/end), not the sensors.
- `include/config.h` holds the DEFAULT pins and the hard limits. It is the
  single source of truth for those. Runtime pin changes go through
  Settings → Hardware and are validated by `src/config/pin_rules.cpp`.

## Architecture
- `src/control/`, `src/calibration/`, `src/util/`, plus
  `config/pin_rules.cpp`, `motors/motor_math.cpp`,
  `sensors/slf3s_protocol.cpp`, `sensors/mprls_protocol.cpp`,
  `sensors/sim_plant.cpp`: **pure logic, no
  Arduino includes**, unit tested natively (`pio test -e native`). Keep it that
  way.
  - `RunManager`: per-channel state machine (IDLE/RUNNING/PAUSED/CALIBRATING/
    FAULT/ESTOP) and all run modes, watchdog/fault/ceiling rules.
  - `FlowController`: feed-forward through the calibration inverse + PI(D)
    correction in flow units.
  - `CalibrationSweep`, `curve_fit`: sweep state machine, fits + inverses.
- `src/motors/`, `src/sensors/slf3s_sensor.*`: hardware-facing drivers only.
  No control logic.
- `src/app/`: glue. `Channel` connects RunManager ↔ MotorDriver ↔
  FlowSensor; `app.cpp` owns the mutex, the tasks, history, events and storage
  jobs.
- `src/net/`: Wi-Fi, REST API (`/api/*`), WebSocket telemetry (`/ws`).
- `data/www/`: vanilla-JS web UI (no build step, uPlot vendored, no CDN).
- Tasks: the real-time task (sensors, run logic, ramps) and the step ISRs run
  on **core 1**; Wi-Fi, AsyncTCP callbacks, telemetry and the storage task run
  on **core 0**. One FreeRTOS mutex guards shared state and is held < ~1 ms.
  **Never do flash writes or blocking work in AsyncWebServer callbacks or with
  the mutex held**; queue an `app::Job` for the storage task instead.

## Safety-relevant conventions (please preserve these when editing)
- Setpoints are **rejected, never clamped**: `FlowController::setSetpoint()`
  and `RunManager::start()` refuse NaN/inf/out-of-range values (fail closed,
  `isSafe() == false`, command 0). Do not "helpfully" auto-clamp without
  discussing it. For a device meant to deliver drug into a body, failing closed
  is the correct default. (Clamping the *actuator command* to
  [minCommand, maxCommand] with anti-windup is fine; that's saturation.)
- `MAX_FLOW_RATE_ML_HR` / `MIN_FLOW_RATE_ML_HR` in `config.h` are hard
  firmware ceilings independent of any smart logic. Every flow-setpoint mode
  is bounded by them, and the measured-flow ceiling trip
  (`FLOW_CEILING_TRIP_*`) bounds manual and calibration modes. New modes must
  respect both.
- The motor driver enforces the profile's `maxCommand` itself (defence in
  depth).
- Pressure protection: when a channel's occlusion limit is set, over-limit
  gauge pressure, sensor saturation or a pressure-sensor fault pauses or stops
  runs in every mode (fail closed). Keep it that way.
- E-STOP latches until explicitly reset. Nothing auto-resumes after E-STOP,
  FAULT, watchdog stop or reboot. Never persist run state.
- Keep control logic out of `motors/` and hardware calls out of `control/`.
  This is what keeps the safety-critical math testable without hardware.

## Useful commands
- Build: `pio run -e esp32dev` (real) / `pio run -e esp32dev_sim` (simulated pumps + sensors)
- Upload firmware + UI: `pio run -e esp32dev -t upload && pio run -e esp32dev -t uploadfs`
- Unit tests (no hardware): `pio test -e native`
- UI without any board: `python3 tools/mock_server.py` → http://localhost:8080 (approximation, not the firmware)
- Headless CSV logging: `python3 tools/logger.py --host 192.168.4.1`
