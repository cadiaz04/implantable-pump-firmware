# API

Base URL: `http://192.168.4.1` (AP) or `http://biopump.local`. All bodies are
JSON. Errors return `{"ok": false, "error": "…"}` with a 4xx/5xx status.
Channel numbers in URLs are **1-based** (`/api/channels/1/…`). Flow is in
**µL/min**, volume in **µL**, pressure in **kPa** (gauge unless named
`…Abs…`), time in **s** unless the field name says otherwise.

## Status

| Method | Path | Notes |
|---|---|---|
| GET | `/api/state` | Full snapshot: `sim`, `version`, `time{epochMs,synced}`, `estop`, `limits`, `settings` (incl. Wi-Fi status), `channels[]` (`hw`, `profile` without calibration points + `activeCalibration`, `status` as below), `signals[]`, `history{capacity,size,periodMs}` |
| GET | `/api/events` | Recent events `[{seq,t,ch,level,msg}]` (level 0 info, 1 warn, 2 alarm; ch 0 = system) |
| POST | `/api/hb` | Watchdog heartbeat (for REST-only clients) |
| POST | `/api/time` | `{"epochMs": 1793600000000}`: sets the wall clock (the ESP32 has no RTC) |
| POST | `/api/reboot` | Only when all channels are idle |

## Settings

| Method | Path | Body |
|---|---|---|
| GET | `/api/config` | → `{config:{watchdogS, telemetryHz, flowUnit, sensorFaultAction, hostname, wifi:{ssid, passwordSet, apSsid, apIp, staConnected, staIp, apClients}}}` |
| PUT | `/api/config` | any of `watchdogS` (2-600), `telemetryHz` (0.5-20), `flowUnit` (`"uL/min"`/`"mL/min"`), `pressureUnit` (`"kPa"`/`"psi"`/`"mmHg"`/`"cmH2O"`, display default only; data stays kPa), `sensorFaultAction` (`"pause"`/`"stop"`, also used for occlusion), `wifi:{ssid, password}`, `apPassword` (8-63 chars, applies after reboot) |
| GET | `/api/hardware` | → `channels[]` `{driver, pins[], pinLabels[], requiredPins, enActiveLow, sensor, pressure, sda, scl, commandUnit, profile}` plus `driverTypes[]` with labels and default pins |
| PUT | `/api/hardware` | `{"channels":[{driver, pins[], enActiveLow, sensor:"slf3s"|"none", pressure:"mprls"|"none", sda, scl, profile?}, …]}`: one entry per channel. Rejected while any channel is active (409). Invalid pins → 400 with `pinErrors:[{channel, field, gpio, reason}]` |

Driver ids: `uln2003` (pins IN1-IN4), `stepdir` (STEP, DIR, EN?), `drv8871`
(IN1, IN2), `pwmdir` (PWM, DIR, EN?). `-1` = not connected (optional pins only).

## Pump profiles

| Method | Path | Notes |
|---|---|---|
| GET | `/api/pumps` | List `[{id, name, pumpType, driverType, family, calibrations, activeCalibration}]` |
| POST | `/api/pumps` | Create or import (body = profile JSON; a new id is assigned if missing or taken) → `201 {id}` |
| GET | `/api/pumps/{id}` | Full profile JSON (the export format) |
| PUT | `/api/pumps/{id}` | Replace (validated). Applied live to channels using it |
| DELETE | `/api/pumps/{id}` | 409 if assigned to a channel |
| POST | `/api/pumps/{id}/duplicate` | Optional `{name}` → `201 {id}` |

Profile JSON:

```json
{
  "id": "28byj48-syringe", "name": "28BYJ-48 Syringe pump (example)",
  "pumpType": "syringe", "notes": "", "fluid": "water",
  "motor": {"driverType": "uln2003", "stepsPerRev": 4096, "microstep": 1, "gearRatio": 1,
            "maxCommand": 15, "minCommand": 0, "accel": 10, "invertDir": false,
            "decay": "fast", "minDuty": 0, "kickMs": 0, "kickDuty": 80, "releaseWhenStopped": true},
  "syringe": {"innerDiameter_mm": 4.78, "leadscrewPitch_mm": 0.8},
  "pid": {"kp": 0.3, "ki": 0.05, "kd": 0, "loopHz": 1, "filterWindowS": 10, "filter": "ma",
          "outputLimitPct": 100, "integralLimitPct": 100},
  "calibrations": [{
    "id": "c1793600000", "name": "water, 2 mm tubing", "timestamp": 1793600000,
    "fluid": "water", "tubingNote": "", "active": true,
    "fitType": "linear", "coefficients": [c0, c1, c2], "lookup": {"x": [], "y": []},
    "xMin": 0, "xMax": 10, "r2": 0.999, "rmse": 1.2, "maxErrPct": 2.1, "nonMonotone": false,
    "points": [{"command": 0, "flow": 0.4, "ref": null, "std": 5.1, "temp": 23.0, "n": 3000, "pass": 0, "status": "ok"}]
  }]
}
```

`flow = c0 + c1·x + c2·x²` (µL/min, x = command). `maxCommand` is in RPM
(steppers, ≤ 600) or duty % (DC, ≤ 100) and is enforced in the motor driver.

## Channels

| Method | Path | Body |
|---|---|---|
| POST | `/api/channels/{n}/run` | see below. Also used for live setpoint changes |
| POST | `/api/channels/{n}/stop` | Stops (and acknowledges FAULT; a second stop clears latched alarms) |
| POST | `/api/channels/{n}/pause` / `resume` | Resume is refused while sensor fault / air-in-line persists in feedback modes |
| POST | `/api/channels/{n}/volume-reset` | Reset the channel's volume totalizer |
| POST | `/api/channels/{n}/profile` | `{"id": "…"}`: channel must be idle; profile must match the driver family |
| POST | `/api/estop` | E-STOP all channels (latches) |
| POST | `/api/estop/reset` | Clear E-STOP; channels go IDLE, nothing restarts |

Run bodies (`keepRunning: true` = don't stop on heartbeat loss):

```json
{"mode": "manual",      "command": 5, "reverse": false, "keepRunning": false}
{"mode": "closed_loop", "flow_uLmin": 62.5, "initialCommand": 3}
{"mode": "open_loop",   "flow_uLmin": 62.5}
{"mode": "volume",      "volume_mL": 15, "duration_h": 4, "openLoop": false, "keepRunning": true}
{"mode": "program",     "program": {"loop": false, "steps": [
    {"kind": "flow", "value": 62.5, "durationS": 600},
    {"kind": "manual", "value": 2.0, "durationS": 60},
    {"kind": "flow_ol", "value": 30, "durationS": 300}]}}
```

Alternatives accepted: `flow_mLmin`, `volume_uL`, `duration_s`.
`initialCommand` is only needed for feedback modes when the pump has no
usable calibration (and no syringe geometry). Out-of-range setpoints are
**rejected** (409), never clamped.

## Sensors

| Method | Path | Body |
|---|---|---|
| GET | `/api/sensors/{n}` | `{sensor:{type, status, flow_uLmin, flow_avg_uLmin, flow_std, temp_C, flags, zeroOffset_uLmin, zeroing, windowS, errors:{crc,i2c,consecutive,reinits}}}` |
| POST | `/api/sensors/{n}/zero` | `{"seconds": 10}` tare (channel idle) · `{"clear": true}` · `{"offset_uLmin": 3.2}` (manual) |
| POST | `/api/sensors/{n}/pressure-zero` | `{"seconds": 5}` pressure tare (pump stopped, port open to air) · `{"clear": true}` (back to standard atmosphere) · `{"offset_kPa": 100.9}` (manual absolute atmosphere) |
| POST | `/api/sensors/{n}/pressure-alarm` | `{"kPa": 20}` occlusion limit, gauge kPa (0 = off, max 160) |

`GET /api/sensors/{n}` also returns `pressure:{type, pressurePresent, pressure_kPa, pressureAbs_kPa, pressureZero_kPa (null = untared), pressureTared, pressureSensor, pressureSaturated, pressureStatusByte, pressureTaring, pressureTareProgress, pressureAlarm_kPa, pressureErrors:{frame,i2c,consecutive,reinits}}`.

## Calibration

| Method | Path | Body |
|---|---|---|
| POST | `/api/calibration/{n}/start` | `{commands:[…]}` **or** `{min, max, points}`; `settleS` (60), `averageS` (60), `repeats` (1), `upDown` (false), `zeroFirst` (true), `zeroS` (30), `zeroSettleS` (15), `reverse`, `keepRunning` (true), `steadyWindowS` (20), `steadyTimeoutS` (120), `maxSlopePctPerMin` (3), `maxStdPct` (25), `stdFloor_uLmin` (20), `slopeFloor_uLmin_per_min` (3). Max 20 commands, 40 points total |
| POST | `/api/calibration/{n}/abort` | Stops the motor; data kept |
| POST | `/api/calibration/{n}/point` | `{index, flow_uLmin}` or `{index, volume_uL, duration_s}` or `{index, clear:true}` |
| POST | `/api/calibration/{n}/retry` | `{index}`: re-measure one point |
| POST | `/api/calibration/{n}/accept` | `{fitType:"linear"|"quadratic"|"lookup", name, tubingNote}`: saves to the profile as active |
| POST | `/api/calibration/{n}/discard` | Clears sweep data |
| GET | `/api/calibration/{n}/status` | `{status:{phase, running, index, phaseElapsed, phaseDuration, remainingS, zeroOffset, commandUnit, config, validPoints, points:[{i, command, pass, status, flow, std, temp, ref, n}], fits:{linear|quadratic|lookup: {fitType, coefficients, lookup?, r2, rmse, maxErrPct, nonMonotone, usable, residuals[]} or {error}}}}` |

Point status: `pending`, `active`, `ok`, `not_steady`, `air`, `sensor`.
Phases: `idle`, `zero_settle`, `zero`, `settle`, `steady`, `average`,
`done`, `aborted`.

## Data

| Method | Path | Notes |
|---|---|---|
| GET | `/api/history?ch=n` | 1 Hz ring buffer (≈24 min): `{synced, cols:["t","flow","setpoint","command","temp","state","pressure"], rows:[[t_ms,…],…]}`. `t` is epoch ms when the clock was synced, otherwise uptime ms |
| GET | `/api/log.csv` | Both channels' history as CSV |
| DELETE | `/api/history` | Clear the ring buffer |
| POST | `/api/sim/{n}` | **sim build only**: `{bubble: seconds}`, `{fault: true|false}`, `{gainScale: 0.8}`, `{occlusion: true|false}`, `{pressureFault: true|false}` |

## WebSocket `/ws`

**Server → client**

* `{"type":"hello","sim":…,"version":…,"clientId":…}` on connect.
* `{"type":"tel", "t": epochMs, "up": uptimeMs, "synced": bool, "estop": bool, "hbAge": s, "ch": [...], "sig": {...}}`
  at the telemetry rate (default 5 Hz). Each `ch[i]`:
  `state, mode, command, commandActual, commandUnit, stepRate_Hz, setpoint,
  manualCommand, reverse, flow_uLmin, flow_avg_uLmin, flow_std, temp_C,
  volume_uL, volumeEstimated, runVolume_uL, plannedVolume_uL, targetVolume_uL,
  eta_s, elapsed_s, flags (sensor signaling flags), alarms (bitmask),
  sensor ("OK"|"INIT"|"FAULT"|"ABSENT"), errors{crc,i2c,consecutive,reinits},
  zeroOffset, zeroing, zeroProgress, keepRunning, programStep, profileId,
  profileName, profileMismatch, hasCalibration, ff, corr, cal{phase,index,points,elapsed,duration}`,
  and, when the channel has a pressure sensor, `pressure_kPa` (gauge, 1-s avg),
  `pressureAbs_kPa, pressureTared, pressureZero_kPa, pressureSensor,
  pressureSaturated, pressureTaring, pressureTareProgress, pressureAlarm_kPa,
  pressureErrors{frame,i2c,reinits}`.
  `sig` maps every registered signal id (see `/api/state.signals`) to its value:
  `f1` flow, `fa1` flow avg, `sp1` setpoint, `c1` command, `t1` temperature,
  `v1` volume, `p1` gauge pressure (kPa, only with a pressure sensor), plus any
  extra registered sensors.
* `{"type":"event","seq","t","ch","level","msg"}`.
* `{"type":"ack","id","ok","error"}` replies to control messages.

Alarm bits: 1 air-in-line, 2 sensor fault, 4 sensor high-flow flag, 8 flow
ceiling tripped, 16 watchdog stop, 32 E-STOP, 64 no sensor, 128 over-pressure
(occlusion), 256 pressure-sensor fault.

**Client → server**

* `{"type":"hb"}` heartbeat (send every 1-2 s; any client keeps the watchdog fed)
* `{"type":"time","epochMs":…}`
* `{"type":"estop"}`, `{"type":"estop_reset"}`
* `{"type":"run"|"stop"|"pause"|"resume"|"volume-reset"|"profile", "ch": 1, "id": "optional-correlation-id", …same fields as REST…}`

## Example (curl)

```bash
H=http://192.168.4.1
curl -s $H/api/state | jq '.channels[0].status.flow_avg_uLmin'
curl -s -XPOST $H/api/channels/1/run -d '{"mode":"volume","volume_mL":15,"duration_h":4,"keepRunning":true}'
curl -s -XPOST $H/api/estop
```
