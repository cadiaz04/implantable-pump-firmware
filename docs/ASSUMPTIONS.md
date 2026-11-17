# Assumptions, defaults and deviations

Everything below was decided where the build spec was silent or ambiguous,
or where it conflicted with the project's existing safety conventions
(CLAUDE.md). Each item says what was chosen and why.

## 1. Deviations from the spec

| Spec | What was built | Why |
|---|---|---|
| FastAccelStepper for STEP/DIR; AccelStepper or own code for ULN2003 | **One in-house step engine** (`src/motors/step_engine.cpp`) for both: ESP32 timer-group timers at 10 MHz, IRAM interrupt, event scheduled exactly when due | Pins must be remappable **at runtime**; FastAccelStepper cannot release and re-attach steppers to new pins without a reboot. The IRAM ISR also keeps stepping during flash writes (saving a profile mid-run). Measured design limit 20 kHz, jitter ≈ ISR latency (~1 µs). Uses the IDF 4.4 legacy timer API, so the platform is **pinned to espressif32 6.12.0** (Arduino core 2.0.17). |
| Web files in `data/` | Web files in **`data/www/`**, served from `/www` | Keeps the UI separate from `/pumps/*.json` in the same LittleFS. |
| Layout lists `config/ motors/ … util/` | Added **`src/app/`** (channel glue, tasks, history, events) and kept **`include/config.h`** | `control/` must stay Arduino-free for native tests, so the glue that touches drivers and sensors needs its own home. `config.h` stays the single source of truth for default pins and hard limits (existing project convention). |
| `esp32s3` environment (existing) | **Removed** | Spec asks for three envs; pin validation is specific to the classic ESP32 (S3 has a different flash/strapping map). Recoverable from git history. |
| Partition table not specified | **`huge_app.csv`** (3 MB app, ~900 KB LittleFS, **no OTA**) | The default table left 17 % app headroom. OTA was "nice to have". |
| `docs/README.md` | Root **`README.md`** | GitHub/VS Code convention; docs/ holds the other four. |

## 2. Safety decisions

* **Reject, don't clamp** (existing project convention, kept). Flow setpoints
  outside `[MIN_FLOW_RATE_ML_HR, MAX_FLOW_RATE_ML_HR]` (0-50 mL/h = 0-833
  µL/min), NaN or inf are **rejected** by `FlowController::setSetpoint()` and
  `RunManager::start()`. Manual commands outside `[0, maxCommand]` or between
  0 and `minCommand` are rejected. The motor driver *also* clamps to
  `maxCommand` (defence in depth; the spec's "hard max-command clamp in the
  driver").
* **Actuator saturation is clamping**, not setpoint clamping: in closed loop
  the *command* is limited to `[minCommand, maxCommand]` with integrator
  anti-windup. That's standard control practice and doesn't change the
  setpoint the user asked for.
* **Firmware flow ceiling in every mode.** CLAUDE.md requires every new
  control mode to be bounded by `MAX_FLOW_RATE_ML_HR`. Manual and calibration
  modes are in command units, so they are bounded by measurement instead: if the
  averaged measured flow exceeds `MAX × 1.10` for 5 s while the motor is active,
  the channel goes to **FAULT** (motor off, acknowledge with Stop). Consequence:
  a fast DC pump swept to 100 % duty will trip this. Raise the constant in
  `include/config.h` deliberately if your bench work needs more.
* **Air-in-line policy:** feedback modes (closed loop, closed-loop volume,
  program flow steps) **pause** by default (configurable: stop). **Manual and
  open-loop runs only raise the alarm**, because priming the line is done in
  manual mode and is *supposed* to push air. During a calibration sweep, air
  invalidates the point being averaged. The flag is debounced for 0.5 s.
* **Sensor FAULT:** pauses or stops feedback runs, aborts calibration sweeps,
  and only alarms in manual/open-loop. Closed-loop cannot start with a faulted
  or absent sensor.
* **Watchdog** is global: any connected client's heartbeat feeds it. It trips
  only when **no** browser is sending heartbeats. Starting a run counts as a
  heartbeat. Browsers throttle timers in background tabs, so the UI also sends a
  heartbeat in reply to every telemetry frame (≤ 1/s). REST-only clients can
  use `POST /api/hb`. `tools/logger.py` sends **no** heartbeats unless
  `--heartbeat` is given.
* **Calibration sweeps default to "keep running if disconnected"** (the spec
  wants a locked phone not to break a sweep). The checkbox can turn it off.
* **E-STOP** is sent by the UI over both WebSocket and REST (whichever arrives
  first wins); it is logged once. It latches; reset returns channels to IDLE.
* **Boot:** all motors off, drivers disabled where an EN pin exists. Nothing
  about runs is persisted (only settings, hardware, profiles, zero offsets).
* **No authentication.** Anyone who can join the AP (WPA2 password) or the same
  network can control the pumps. Acceptable for a bench tool; change the AP
  password from the default `biopump123`. CORS is open (`*`) so scripts on
  laptops can call the API.

## 3. Control design choices

* **PID in flow units.** `command = f⁻¹(setpoint + correction)`, where the
  correction (µL/min) comes from the PID on the filtered flow. That makes Kp
  dimensionless and Ki 1/s, independent of each pump's µL/min-per-unit slope.
  Defaults: Kp 0.3, Ki 0.05 s⁻¹, Kd 0, loop 1 Hz, derivative on measurement.
* **PID limits** (`outputLimitPct`, `integralLimitPct`) are **% of the
  setpoint**, with a floor of 20 µL/min, default **100 %**. A 50 % default
  failed the no-calibration test (an initial-command guess 36 % off couldn't
  be corrected).
* **No calibration:** closed loop uses a proportional model `flow = k·command`
  with `k = setpoint / initialCommand`, or the syringe's theoretical µL/min/RPM
  if geometry is set. Open-loop needs a calibration or syringe geometry.
* **Measurement filter:** moving average (default) or EMA (τ = window/2) over
  the profile's `filterWindowS` (10 s default; 20 s for the DC diaphragm
  example). The same window drives the dashboard's "Flow (avg) ± σ".
* **Rolling statistics** use 60 time buckets, so windows of many minutes cost
  ~1.5 KB each.
* **Volume totalizer** integrates the tare-corrected sensor flow
  (trapezoid) while the motor is commanded, plus 15 s after it stops (sensor
  lag). If the sensor is absent or faulted it integrates the calibration's (or
  syringe's) estimate and marks the volume "estimated" (`*` in the UI).
* **Volume mode:** target flow = V/T. *Planned volume* counts only RUNNING
  time (excludes pauses). ETA = remaining volume / target flow.
* **DC deadband:** commands between 0 and `minDuty` drive at 0. An extra
  profile field **`kickDuty`** (default 80 %, clamped to maxCommand) is used
  for the start kick (`kickMs`), because the spec named the duration but not the
  level. `minCommand` (default 15 % in the example) is the closed-loop lower
  clamp.
* **Stepper units:** command = RPM of the **driven shaft**;
  steps/s = rpm × stepsPerRev × microstep × gearRatio / 60, with
  `gearRatio` = motor revs per driven-shaft rev (1 = direct). For the 28BYJ-48
  the 4096 half-steps already include its internal gearbox.
* **Reversing** ramps to zero first, then changes direction.
* **Program mode:** steps `manual` (command), `flow` (closed loop), `flow_ol`
  (open loop); max 16; optional loop. All steps are validated before starting.

## 4. Calibration choices

* Default sweep: 10 points from minCommand to maxCommand, 60 s settle, 60 s
  average, 1 repeat, ascending, zero first (15 s settle + 30 s average).
* Steady state: 20 s window; |slope| ≤ max(3 %/min of mean, 3 µL/min/min) and
  std ≤ max(25 % of mean, 20 µL/min); timeout 120 s → the point is measured anyway
  and flagged `not_steady` (still used in fits).
* Max 20 distinct commands, 40 points including repeats and descending pass.
* **maxErrPct** ignores points below 2 % of the largest flow (a % error at zero
  flow is meaningless).
* **Lookup fits** average duplicate commands, then enforce monotonicity with
  pool-adjacent-violators and set `nonMonotone`. The inverse is a bisection over
  the increasing part. Quadratic inverses are restricted to the increasing side of
  the vertex. A fit that doesn't increase within [min, max] command is rejected
  on accept.
* Up to **10 calibrations per profile**; the oldest inactive one is dropped.
* Manual references accept `flow_uLmin`, or `volume_uL` + `duration_s`. The UI
  helps with "µL collected in N s"; density conversion is left to the user
  (1 mg ≈ 1 µL water).

## 5. Hardware / pins

* **Strapping pins (0, 2, 5, 12, 15) are rejected outright** rather than
  warned about: the simplest safe rule for a team of non-specialists. GPIO 2 is
  still used for the on-board LED (output only after boot).
* UART0 pins 1/3 are rejected (USB serial).
* Ch 2's default sensor bus uses GPIO 32/33 (ADC1). Future analog sensors
  should use 34-39.
* Defaults: Ch 1 = ULN2003 (28BYJ-48 syringe example), Ch 2 = DRV8871
  (DC diaphragm example), so two different pump profiles run out of the box.
* **Profile ↔ channel compatibility is by family** (stepper vs DC). The
  channel's *hardware* driver type is what drives the pins; the profile's
  `driverType` is informational within a family. Mismatches are refused.
* Generic PWM+DIR: DIR LOW = forward; optional EN driven HIGH while running.
* STEP/DIR EN defaults to active-low (A4988/DRV8825/TMC2209).
* Hardware changes require every channel to be idle, and are applied by the
  real-time task (drivers and sensor buses are torn down and rebuilt), so no
  recompile or reboot is needed.

## 6. SLF3S-1300F: datasheet checks

All constants live in `src/sensors/slf3s_protocol.h`.

| Item | Spec value | Checked against | Result |
|---|---|---|---|
| I²C address | 0x08 | Sensirion `arduino-i2c-slf3x` driver | ✔ |
| Start continuous, water / IPA | 0x3608 / 0x3615 | driver | ✔ |
| Stop continuous | 0x3FF9 | driver | ✔ |
| Product ID | 0x367C then 0xE102 | driver | ✔ (only the product number is read; it is not checked) |
| Flow scale (1300F) | raw / 500 → mL/min | driver | ✔ |
| Temperature scale | raw / 200 → °C | driver | ✔ |
| Flags | bit 0 air-in-line, bit 1 high flow, bit 5 exp. smoothing | driver | ✔ |
| Timing after start | ~60 ms | driver: first result after 12 ms, small accuracy deviations until 60 ms | ✔ consistent (firmware waits 60 ms) |
| CRC-8 | poly 0x31, init 0xFF | Sensirion standard (datasheet example 0xBEEF → 0x92 is a unit test) | ✔ |
| Soft reset | general call 0x0006 | spec only | ⚠ not independently verified; harmless if ignored (sensor is alone on its bus) |
| Power-up time | 25 ms | spec only | ⚠ not independently verified |
| Accuracy ±5 % m.v. / ±0.05 mL/min, repeatability, ±40 mL/min FS, 3.2-3.8 V | spec | could not extract the datasheet PDF text here | ⚠ taken from the spec; confirm against the datasheet revision you have |
| Connector pinout 1 n.c., 2 SDA, 3 VDD, 4 GND, 5 SCL, 6 n.c. | — | distributor copy of the datasheet | ✔ (check your cable) |

No mismatches were found. Implementation choices:

* Bus speed 400 kHz, Wire timeout 10 ms, poll every **20 ms (50 Hz)**.
* **FAULT after 10 consecutive** CRC/I²C failures (≈ 0.2 s). While faulted the
  sensor is re-initialised once per second with I²C bus recovery (9 SCL
  clocks + STOP), then soft reset and start.
* Saline: the water command is always used; the UI warns about fluid-dependent
  error.

## 7. Networking, data, UI

* AP SSID `BioPump-XXXX` (last two MAC bytes), password `biopump123`,
  192.168.4.1; mDNS `biopump.local`; STA credentials in NVS. Changing the AP
  password takes effect after reboot. WPA2-Enterprise is not supported.
* Telemetry 5 Hz (0.5-20 Hz configurable). Up to 6 WebSocket clients are
  allowed (the AP itself handles about 4 stations).
* **History ring:** 1 Hz, sized to `HISTORY_RAM_BUDGET_BYTES` = 64 000 B →
  **1454 entries ≈ 24 min** with 2 channels (44 B/entry: 1-s mean flow,
  setpoint, command, temperature, gauge pressure, state, alarm bits). Allocated
  at boot. Adding pressure cut this from ≈30 min; raise the budget if heap
  headroom allows (`heapFree` in `/api/state`).
* API channel numbers are **1-based**, matching the UI labels.
* Time: browsers send their clock on connect. Until then, timestamps are uptime
  and the history endpoint reports `synced: false` (the UI then skips backfill
  to avoid mixing time bases).
* Flow units: µL/min default with a mL/min toggle (stored per browser, plus a
  server-side default).
* The browser session log records every telemetry frame (5 Hz). A 4-hour run is
  about 72 k rows, fine for a laptop. Use `tools/logger.py` for unattended runs.
* uPlot 1.6.32 is vendored with its MIT license; no CDN or internet is used at
  runtime.

## 8. Simulation model (SIM_MODE)

* Stepper channels: gain = 0.92 × the profile's theoretical syringe
  µL/min/RPM (20 if no geometry), slight negative curvature, τ = 4 s lag, 2 s
  dead time, 5 µL/min noise, +8 µL/min sensor offset (+3 per channel).
* DC channels: 12 % deadband, 6 µL/min per % + 0.02 per %², ±25 % pulsation at
  0.04 Hz per % duty, τ = 3 s, −5 µL/min offset.
* Bubble injection reads ~10 % of flow with the air flag set. Fault injection
  produces CRC errors → FAULT after 10 polls.
* `tools/mock_server.py` is a separate, much cruder Python imitation for UI
  development only.

## 9. Not verifiable without hardware

Everything below compiles, and the logic behind it is unit tested, but none of it
has run on a real board yet:

* **Step engine timing** on silicon (ISR latency, 10-20 kHz behaviour, Wi-Fi
  coexistence), ULN2003 phase order and torque at speed.
* **LEDC PWM** output and the DRV8871 slow-decay mapping on a scope.
* **SLF3S I²C** traffic, CRC handling on real frames, product-ID read, bus
  recovery after a hung bus.
* **Wi-Fi/AsyncWebServer** behaviour with 4 phones plus the logger at 5 Hz, and
  heap headroom over a 4-hour run (static RAM is 23 % used; check
  `heapFree` in `/api/state`).
* **NVS/LittleFS** atomic-rename behaviour on power loss.
* **mDNS** on your specific networks.
* **MPRLS on the real bus**: the I²C timing with the SLF3S on the same bus,
  conversion pacing, and recovery when one device hangs the bus.
* The **simulator firmware itself** (`esp32dev_sim`) has not been flashed in
  this session (no board was attached); the UI was exercised against
  `tools/mock_server.py` in headless Chrome at desktop and 360 px widths.

## 10. Pressure sensor (SparkFun Qwiic MicroPressure, Honeywell MPRLS0025PA00001A)

Added after the first build, from a suggested addendum. What I checked and
what I changed:

### Datasheet checks (Honeywell MPR Series datasheet)

| Item | Suggested | Datasheet | Result |
|---|---|---|---|
| Part number meaning | 0-25 psi absolute | MPR, **L**ong port, **S**ilicone gel, 0025**PA** = 0-25 psi absolute, 000**0**, **1** = I²C address 0x18, **A** = 10-90 % transfer function | ✔ |
| I²C address | 0x18 | 0x18 (default) | ✔ |
| Measure command | 0xAA 0x00 0x00 | 0xAA followed by 0x00 0x00 | ✔ |
| Read | 1 status + 24-bit pressure | status byte, then 24-bit pressure (4 bytes) | ✔ |
| Conversion time | 5-10 ms | wait ≥ 5 ms, or poll busy, or EOC | ✔ (firmware reads 20 ms after triggering, re-polls if busy) |
| Transfer function | (raw − 1677722)·25 / (15099494 − 1677722) | 10 %/90 % of 2²⁴ = 0x19999A / 0xE66666; P = (Out − Outmin)(Pmax − Pmin)/(Outmax − Outmin) + Pmin | ✔ (datasheet worked example is a unit test) |
| Status bits | power, busy, integrity, saturation | bit 6 power (1 = powered), bit 5 busy, bit 2 memory integrity fail, bit 0 math saturation; bits 7, 4, 3, 1 always 0 | ✔ |
| Supply | 3.3 V | 3.0-3.6 V, abs. max 3.6 V | ✔ |
| Overpressure / burst | — | 60 psi / 120 psi | added to WIRING.md |
| Accuracy | — | ±0.25 %FSS BFSL; **total error band ±1.5 %FSS** after auto-zero (0-50 °C) | see "honest limits" below |
| Media | — | gel versions (S, F, L) are the liquid-media option; wetted: 304 stainless steel, gel | ✔ saline in the line is OK |
| General-call reset | — | not mentioned | led to the bus change below |

No mismatches with the suggestion's protocol details.

### Where I deviated from the suggestion

* **Hand-rolled, non-blocking driver instead of the SparkFun library.** The
  library's `readPressure()` blocks while the sensor converts (~5 ms per read)
  and owns `Wire.begin()`. Both are wrong for the real-time task, which also
  serves the flow sensor. The driver here is pipelined (trigger → read 20 ms
  later → trigger again, 50 Hz, no waiting), and the decoding is in a pure
  module with native tests.
* **The MPR has no CRC**, so garbled frames are caught by checking the
  always-zero bits and the power bit (0x00 and 0xFF frames are rejected).
* **Signal IDs `p1`, `p2`, not `pressure`.** One ID per channel, otherwise two
  sensors collide in telemetry and CSV.
* **Data always in kPa; the unit selector is display-only.** If the logged
  unit followed a UI setting, changing it mid-run would silently corrupt CSV
  logs (the logger reads column units once at start). The selector (kPa / psi
  / mmHg / cmH₂O) converts the dashboard, plot axis and limit entry.
* **The high-pressure alarm acts; it doesn't just warn.** A downstream
  occlusion is the classic infusion-pump hazard, so when a limit is set, gauge
  pressure above it for 1 s **pauses (or stops, per the "On sensor fault"
  setting) runs in every mode, including manual**, aborts calibration
  sweeps, and blocks starting or resuming until pressure falls back. A
  saturated or faulted pressure sensor counts as over-pressure while a limit is
  set (fail closed: it can't prove the line is clear). The limit is
  **off by default** because a sensible value depends on your line and catheter;
  it's per channel, stored in NVS, and set in *Sensor & zero*.
* **Bus ownership moved to the channel.** Both sensors share one `TwoWire`, so
  the channel begins and ends the bus, not the sensors. On a shared bus the
  SLF3S init no longer sends the I²C **general-call reset** (which reaches
  every device, and the MPR datasheet doesn't say how it reacts). It sends
  "stop continuous measurement" instead, which also recovers the sensor after an
  ESP32-only reboot. SLF3S bus recovery (clocking SCL) still runs on the shared
  bus; the MPRLS just sees a few failed reads during it.
* **Tare semantics.** Tare stores the absolute atmosphere (kPa) with the pump
  stopped and the port open to air; gauge = absolute − tare. Before the first
  tare the firmware uses the standard atmosphere (101.325 kPa) and the UI says
  "untared". Tare requires the channel not to be pumping.
* **Defaults:** pressure sensor enabled on **Ch 1 only** (one board bought);
  enable Ch 2 in Settings → Hardware.
* The **AnalogPressureSensor stub was removed.** The generic `SensorRegistry`
  remains for any other future sensor.

### Honest limits of this sensor for this project

* Total error band ±1.5 %FSS = **±0.375 psi ≈ ±2.6 kPa ≈ ±19 mmHg ≈
  ±26 cmH₂O**, even after zeroing. Normal CSF pressure is about 7-15 mmHg, so
  this part **cannot characterise physiological intrathecal pressure**. It is
  well suited to occlusion detection, back-pressure trends and relative changes
  (its resolution is far finer than its accuracy). For CSF-scale measurements,
  use a low-range *gauge* or differential sensor (e.g. an MPR or ABP variant
  rated ±1 psi or less).
* Being absolute, a tare drifts with the weather (up to a few kPa over days).
  Re-tare at the start of each session.
* Reported values are pressure at the sensor; add or subtract the hydrostatic
  head to wherever you care about (≈ 0.098 kPa per cm of water).

### Simulation

`SimPlant` adds an absolute pressure: atmosphere (100.8 kPa, deliberately not
standard so a tare matters) + slow weather drift + 0.02 kPa per µL/min of pump
flow through a 1.5 s line-compliance lag + 0.03 kPa noise. **Occlusion**
injection multiplies the line resistance by 40 and cuts the delivered (measured)
flow to 5 %. Pressure-sensor fault injection is separate from the flow sensor's.
Both are on Settings → Simulator and `POST /api/sim/{n}`.

