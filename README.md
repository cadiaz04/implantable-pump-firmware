# BioPump Controller

ESP32 firmware and web app for bench-testing prototype pumps for an
implantable intrathecal drug-delivery pump (Rice BIOE senior design). It
controls up to **2 pumps** (stepper or brushed DC), **calibrates** them against
a **Sensirion SLF3S-1300F** flow sensor, monitors line pressure with a
**SparkFun Qwiic MicroPressure** (occlusion alarm), runs **closed-loop flow control** and
**"deliver V mL over T hours"** runs, plots everything live and exports CSV.
Everything runs in a browser on a laptop or phone, with no software to install and
no internet needed.

| Doc | What's in it |
|---|---|
| [docs/WIRING.md](docs/WIRING.md) | Pin map, wiring diagrams for every driver and the flow sensor, power notes |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | Wet-calibration procedure and the low-flow accuracy caveats |
| [docs/API.md](docs/API.md) | REST + WebSocket API (for scripts and `tools/logger.py`) |
| [docs/ASSUMPTIONS.md](docs/ASSUMPTIONS.md) | Every default/assumption made, datasheet checks, what is unverified |
| [CLAUDE.md](CLAUDE.md) | Architecture and safety conventions (for humans and AI assistants) |

> **Hardware compatibility:** a **ULN2003** board can only drive a
> **unipolar** stepper such as the 28BYJ-48. It **cannot** drive a bipolar
> NEMA 17. NEMA motors need a **STEP/DIR** driver (A4988, DRV8825, TMC2209).
> The firmware supports all of these; pick the driver per channel in
> Settings → Hardware.

---

## Quick start

Prerequisites: VS Code + the PlatformIO extension (or `pip install platformio`).

```bash
# 1. Firmware
pio run -e esp32dev -t upload
# 2. Web UI (writes data/ to the LittleFS partition)
pio run -e esp32dev -t uploadfs
# 3. Serial log (optional)
pio device monitor
```

> ⚠️ `uploadfs` erases the whole filesystem, **including saved pump profiles
> and calibrations**. Export them first (Pumps → Export). Starter profiles are
> recreated automatically when none exist.

Then:

1. Join the Wi-Fi network **`BioPump-XXXX`** (password `biopump123`; change
   it in Settings → Wi-Fi).
2. Open **http://192.168.4.1** (or **http://biopump.local** where mDNS works).
3. On the Dashboard, pick a pump profile for each channel and press **Start**.

The ESP32 always runs its own access point. It can *also* join an existing
network (Settings → Wi-Fi), and then it is reachable at the IP shown there or
at `biopump.local`.

**Networking, honestly:** campus Wi-Fi often isolates clients from each
other, blocks mDNS, or requires WPA2-Enterprise, which the firmware does not
support. The reliable options are the ESP32's own AP or a phone hotspot that
the ESP32 joins. The ESP32 AP handles **about 4 clients**. A Raspberry Pi 4
can act as the router or run `tools/logger.py` unattended, but **no Pi is
required**.

### Simulation mode (no hardware at all)

```bash
pio run -e esp32dev_sim -t upload && pio run -e esp32dev_sim -t uploadfs
```

Same firmware, but the motors and flow sensors are simulated. Each simulated
pump has flow = k·command (+ curvature, DC deadband), a first-order lag, a dead
time, sensor noise and offset, pulsation for DC diaphragm pumps, and bubble/fault
injection (Settings → Simulator). The UI shows an orange **SIM** badge and
`/api/state` reports `"sim": true`. A bare DevKit with nothing attached is
enough; no GPIO is driven in sim mode.

For UI work on a laptop without any ESP32, `python3 tools/mock_server.py`
serves the UI against a crude Python imitation of the API at
http://localhost:8080. It is **not** the firmware; use the sim build for
anything that matters.

### Tests

```bash
pio test -e native     # 79 host-side unit tests, no hardware
```

They cover CRC-8, SLF3S frame parsing and scaling (including CRC failures), linear,
quadratic and monotone-lookup fits with their inverses, PID anti-windup, steps/s
conversion, volume integration, pin validation, MPRLS frame decoding and the occlusion alarm, the run state machine (watchdog,
E-STOP, faults, volume completion, programs), the calibration sweep (air,
retry, references, tare) and end-to-end **calibrate → hold 62.5 µL/min** runs
against the simulated plant, which hold within ±3 %.

---

## Using the web UI

The **E-STOP** button (bottom right) is on every page. It stops all motors
immediately, disables driver outputs and **latches** until you press *Reset
E-STOP*. Nothing restarts automatically after a reset or a reboot.

**Dashboard.** One card per channel shows pump profile, state, sensor
status, alarms, live readouts (flow ± 1σ noise, instantaneous flow,
command, temperature, delivered volume, ETA) and a live plot (flow, average,
setpoint, command, temperature; 1 min / 10 min / 1 h / all; drag to zoom,
double-click to reset, *Pause plot*). Modes:

| Mode | Setpoint | Notes |
|---|---|---|
| Manual | RPM (steppers) or **Duty %** (DC) | Live changes are ramp-limited. *Reverse* ramps through zero. |
| Closed-loop | flow (µL/min) | Feed-forward from the active calibration + PI correction on the measured flow. Without a calibration, enter an *initial command*. |
| Volume | V mL over T h | The main use case (preset 15 mL / 4 h ≈ 62.5 µL/min). Integrates delivered volume from the sensor, shows delivered vs planned and ETA, and stops at the target. Closed-loop by default; open-loop optional. |
| Open-loop | flow | Command from the calibration inverse, no feedback. |
| Program | list of steps | Command, closed-loop flow or open-loop flow, each with a duration; optional loop. |

Tick **"Keep running if disconnected"** for long runs. Otherwise the run stops
when no browser has sent a heartbeat for the watchdog timeout (default 10 s),
for example when the last tab closes or a phone sleeps. Sensor FAULT, air-in-line
and E-STOP still apply to keep-running runs.

*Sensor & zero* (in each card): **tare** the flow sensor with the pump
stopped and the line wetted. The offset is stored and shown.

**Pressure (Qwiic MicroPressure):** the sensor reads *absolute* pressure, so
with the pump stopped and its port open to air press **Tare pressure**. The
card then shows **gauge** pressure (choose kPa / psi / mmHg / cmH₂O in
Settings). Set an **occlusion alarm** limit per channel: pressure above it for
1 s pauses (or stops) any run, including manual. The limit is off until you
set one. Accuracy is about ±2.6 kPa (±19 mmHg), fine for occlusion and
back-pressure trends but not for absolute CSF pressure; see
[docs/ASSUMPTIONS.md §10](docs/ASSUMPTIONS.md).

**Calibrate.** Configure the sweep (points, settle time, averaging time,
repeats, optional descending pass, zero first), start it, watch the
flow-vs-command scatter grow, enter **manual references** (µL collected in N
seconds, or a flow) for any point, retry bad points, compare linear,
quadratic and lookup fits (R², RMSE, max error, residuals), then **Accept**
to save the fit to the pump profile as the active calibration. The sweep runs
on the ESP32, so a locked phone cannot break it. See
[docs/CALIBRATION.md](docs/CALIBRATION.md).

**Pumps.** Create, duplicate, edit, import, export or delete pump profiles:
driver type, steps/rev, microstepping, gear ratio, max/min command,
acceleration, DC deadband and start kick, syringe geometry (shows theoretical
µL/min per RPM), PID gains, averaging window, and which calibration is active.

**Settings.** Hardware drivers and pins per channel (validated: no flash,
strapping, UART or input-only pins, no conflicts; applied without recompiling
while channels are stopped), flow units, watchdog timeout, telemetry rate,
pause-vs-stop on sensor faults, Wi-Fi, time sync, reboot.

**Data.** Download the browser's session log (every telemetry frame, all
signals, state and air-in-line columns) or the ESP32's ~30-minute history
buffer. For unattended multi-hour runs use the logger:

```bash
python3 tools/logger.py --host 192.168.4.1 --out run1.csv   # stdlib only; Mac/Linux/Pi
```

The ESP32 has no real-time clock. The browser sends its time on connect, so CSV
timestamps are wall-clock once any browser has connected (uptime before that).

---

## Adding sensors and pumps later

* **New sensor:** write one `Sensor` subclass and register it in `main.cpp`
  (`g_sensors.add(&mySensor)`). Its signals appear automatically in
  telemetry, live plots and CSV columns. (The per-channel flow and pressure
  sensors are built in and chosen in Settings → Hardware.)
* **More channels:** raise `MAX_CHANNELS` in `include/config.h` and add
  default pins. The ESP32 has two I²C controllers and every SLF3S uses the
  fixed address 0x08, so a third flow sensor needs a **TCA9548A I²C
  multiplexer** (one sensor per mux channel). The step generator has timers
  for up to 2 steppers; 4 are possible with timer group 1.

## Project layout

```
include/config.h      defaults + hard safety limits (single source of truth)
src/main.cpp          setup: settings → filesystem → Wi-Fi → HTTP/WS → tasks
src/app/              channel glue (motor + sensor + run manager), tasks, history, events
src/config/           pin rules (validation), NVS settings
src/motors/           MotorDriver + ULN2003, STEP/DIR, DRV8871, PWM+DIR, sim; step engine (timer ISR)
src/sensors/          Sensor/FlowSensor/PressureSensor interfaces, SLF3S + MPRLS drivers and protocols, sim plant
src/control/          RunManager state machine, FlowController (FF + PI), PID, watchdog
src/calibration/      CalibrationSweep, curve fits + inverses
src/pumps/            PumpProfile model, JSON, LittleFS store
src/net/              Wi-Fi/mDNS, REST API, WebSocket telemetry
src/util/             CRC-8, window statistics, totalizer, ring buffer
data/www/             web UI (index.html, app.js, style.css, vendored uPlot)
test/                 native unit tests (pio test -e native)
tools/                logger.py (CSV logger), mock_server.py (UI dev without hardware)
docs/                 WIRING, CALIBRATION, API, ASSUMPTIONS
```

## Troubleshooting

| Symptom | Check |
|---|---|
| Page says "Not found… did you run uploadfs?" | Run `pio run -e esp32dev -t uploadfs`. |
| Sensor shows **FAULT** | 3.3 V (never 5 V) on VDD, SDA/SCL not swapped, 4.7 kΩ pull-ups, correct bus pins in Settings → Hardware. The error counters are in *Sensor & zero*. |
| **Pressure FAULT** | Qwiic wires on the *same* SDA/SCL/3V3/GND as that channel's flow sensor; pressure sensor enabled in Settings → Hardware. Integrity failures need a power cycle. |
| Pressure reads ≈ +100 kPa or "untared" | Tare it with the pump stopped and the port open to air. |
| Run paused with "over-pressure" | Check for a kinked or blocked line. Resume works once pressure drops below the limit. |
| Flow reads non-zero with the pump stopped | Tare it (Sensor & zero). Check for bubbles; thermal sensors drift with air. |
| Closed-loop oscillates | Increase the averaging window, lower Ki (Pumps → profile). Pulsatile pumps need windows of several cycles. |
| Run stops after ~10 s | No heartbeat: the tab was closed or the phone slept. Use *Keep running if disconnected*. |
| Run refuses to start: "outside firmware limits" | Flow setpoints are bounded by `MAX_FLOW_RATE_ML_HR` (50 mL/h ≈ 833 µL/min) in `include/config.h`. |
| Channel went to FAULT | Measured flow stayed above the firmware ceiling for 5 s. Press Stop to acknowledge. |
| 28BYJ-48 buzzes but doesn't turn | Reduce max RPM (≈15) or acceleration; check the ULN2003 board has 5 V. |
| `biopump.local` doesn't resolve | Use the IP. Windows and Android mDNS support varies; campus networks often block it. |
| Can't change hardware | Stop all channels first (Settings → Hardware refuses while a channel runs). |
