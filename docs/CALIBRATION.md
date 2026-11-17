# Wet calibration procedure

Goal: measure **flow = f(command)** for one pump, save it to the pump's
profile, and use its inverse **command = f⁻¹(flow)** for feed-forward
(closed-loop, volume and open-loop modes).

## Read first: what the sensor can and can't do at 62.5 µL/min

SLF3S-1300F (water calibration), per the Sensirion datasheet:

| | |
|---|---|
| Full scale | ±40 mL/min |
| Accuracy | **±5 % of reading or ±0.05 mL/min (50 µL/min), whichever is larger** |
| Repeatability | about ±0.01 mL/min (10 µL/min) |

The main test target, **15 mL in 4 h = 62.5 µL/min**, sits at the **bottom
of the sensor's reliable range**: the accuracy floor (±50 µL/min) is almost as
large as the flow itself. Repeatability is much better than accuracy, which
is what makes this workable:

1. **Tare (zero) the sensor.** With the pump stopped and the line fully
   wetted, average the reading and subtract it. This removes most of the
   offset error. The offset is stored per channel and shown in *Sensor &
   zero*. Re-tare whenever the plumbing, the temperature or the fluid changes.
2. **Average for a long time.** Use 60-300 s per point at low flow (the
   firmware averages raw 50 Hz samples, so a 120 s point is about 6000 samples).
   Steady-state detection waits for the rolling slope and std dev to settle.
3. **Use manual references.** At low flow, a gravimetric reference is the
   truth: weigh what the pump delivers over a timed interval. Enter it per point;
   it overrides the sensor value in the fit, and both are stored.
4. **Read the noise band**, not just the number. The dashboard shows flow ± 1σ
   over the averaging window.

The sensor also **lags** the pump: thermal response plus the dead volume of the
tubing between pump and sensor. That's why every point has a settle time and why
the closed-loop controller runs slowly (1 Hz) with a long filter.

### Fluid: water vs saline

The firmware always uses the sensor's **water** calibration (command
`0x3608`). Saline (0.9 % NaCl) has slightly different thermal properties, so
readings carry some fluid-dependent error that the datasheet does not specify.
Recommended:

* calibrate with the fluid you will actually run (set *Fluid* in the profile),
* and **check the final delivered volume against a collected reference**
  (balance) at the end of every important run.

## Procedure

### 1. Prepare

* Plumb: reservoir → pump → (short tube) → SLF3S → outlet into a collection
  vessel on a balance (0.1 mg or 1 mg resolution). Keep the outlet tip below the
  liquid surface, or cover the vessel, to limit evaporation and drop effects.
* Use the same tubing and outlet you'll use for real runs. Note it in the
  calibration's *tubing note*.
* Let the fluid reach room temperature. Degas it if you can (vacuum, or let it
  stand); dissolved gas comes out as bubbles in thermal sensors.

### 2. Prime and remove bubbles

* Manual mode at a high command until liquid flows out of the outlet without
  bubbles. Air-in-line alarms are shown but don't stop manual runs, which is
  intentional for priming.
* Tap or flick the sensor and tubing to dislodge bubbles. Mount the sensor so
  flow goes upward or horizontally.
* Stop the pump. Wait about 30 s.

### 3. Zero

* *Sensor & zero → Zero / tare now* (10-30 s), or leave **Zero sensor first**
  ticked in the sweep setup (default) and the sweep does it.
* A tare fails (and keeps the old offset) if air-in-line is flagged during it.

### 4. Sweep

Calibrate tab → choose the channel. Defaults: 10 points evenly spaced from min
to max command, 60 s settle + 60 s average, 1 repeat, ascending only, zero
first.

* **Steppers:** command = RPM at the driven shaft. Steps/rev, microstep and
  gear ratio only need to be roughly right, because the calibration absorbs the
  difference (e.g. a 28BYJ-48 at ≈4075.8 vs a nominal 4096 steps/rev).
* **DC motors:** command = PWM **duty %**, since there is no encoder. Start the
  sweep at or just above the deadband (*min duty*). Points below the deadband
  read zero and distort a linear fit; give them a lookup fit or leave them out.
* **Low-flow targets:** concentrate points near your operating range with
  the *command list* field (e.g. `1, 2, 3, 4, 6, 10`) and lengthen averaging.
* **Pulsatile (diaphragm) pumps:** averaging should cover many cycles (≥ 60 s);
  raise *Max std dev %* if steady-state detection keeps timing out.
* Tick **Add descending pass** to see hysteresis (backlash, compliance).
* **Keep running if disconnected** is ticked by default for sweeps.

*Abort* stops the motor and keeps the points measured so far.

Per point the firmware goes *settle → steady-state check → average*. The
steady-state check passes when, over the steady window (20 s):

* |slope| ≤ max(3 %/min × mean, 3 µL/min/min), and
* std dev ≤ max(25 % × mean, 20 µL/min).

If that doesn't happen within the timeout (120 s), the point is still measured
but marked **not steady**. Air-in-line during averaging marks a point
**air** (invalid); use **Retry** for that point. All thresholds are editable
under *Steady-state detection*.

### 5. Manual references (recommended below ~0.5 mL/min)

For any point (after it is measured), run the pump at that command, collect
for a timed interval (≥ 10 min at 60 µL/min gives ≥ 600 mg), then enter
**µL collected** and **seconds**. For water 1 mg ≈ 1 µL (0.998 g/mL at 20 °C;
saline ≈ 1.005 g/mL). The reference replaces the sensor value in the fit;
both are kept and plotted.

### 6. Fit and accept

Compare the fits:

| Fit | Use when |
|---|---|
| **Linear** (default) | Syringe pumps; anything proportional |
| **Quadratic** | Gentle curvature (DC pumps, peristaltic) |
| **Lookup** | Arbitrary monotone shapes; deadbands. Duplicate commands are averaged; non-monotone data is repaired (pool-adjacent-violators) and flagged |

Check **R²**, **RMSE** and **max error %** (computed only over points above 2 %
of the largest flow), and look at the residual plot for structure. The inverse
is only evaluated where the fit is increasing and is clamped to the profile's
[min, max] command. A fit that never increases inside that range is rejected.

*Accept & save* stores the calibration (points, fit, statistics, fluid,
tubing note, timestamp) in the pump profile and makes it **active**. Older
calibrations are kept (up to 10 per profile); choose the active one on the
Pumps tab. *Export CSV/JSON* downloads the raw points and the fit.

### 7. Verify

* Run **Open-loop** at your target flow and check against the balance over
  ≥ 30 min.
* Then run **Closed-loop** / **Volume** and compare the final delivered volume
  (dashboard) with the balance. Expect the sensor-integrated volume to be
  within the sensor's accuracy band; the balance is the reference.

## Closed-loop tuning notes

The controller computes `command = f⁻¹(setpoint + correction)`, where the
correction comes from a PI(D) acting on the **filtered** measured flow. Because
it works in flow units through the calibration, the gains are pump-independent:

* **Kp** (dimensionless, default 0.3), **Ki** (1/s, default 0.05),
  **Kd** (s, default 0).
* Loop rate 1 Hz (0.2-10 Hz). Low flows are slow and noisy, so don't speed it up.
* Averaging window 10 s (20 s for the DC diaphragm example). Longer is
  smoother but slower to react.
* The correction and integrator are clamped to ±100 % of the setpoint, with
  anti-windup when the command saturates at the pump's min/max.
* With **no calibration**, closed-loop still works: enter an *initial
  command* that roughly gives the setpoint, and the PI corrects from there
  (tolerates guesses off by up to about 2×).

In simulation (and in the native tests) this holds 62.5 µL/min within ±3 % on
a pump that is 20 % weaker than its calibration says, and on a pulsatile DC
pump with a deadband.
