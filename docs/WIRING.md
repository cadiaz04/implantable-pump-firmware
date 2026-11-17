# Wiring

Board: **ESP32-WROOM-32 DevKit V1** (30-pin). Every pin below is a *default*;
all of them can be changed at runtime in **Settings → Hardware**.

## Default pin map

| Function | Ch 1 | Ch 2 |
|---|---|---|
| ULN2003 IN1, IN2, IN3, IN4 | 16, 17, 18, 19 | 13, 14, 27, 26 |
| STEP / DIR / EN (A4988, DRV8825, TMC2209) | 16 / 17 / 18 | 13 / 14 / 27 |
| DRV8871 IN1 / IN2 | 16 / 17 | 13 / 14 |
| Generic H-bridge PWM / DIR / EN (L298N, TB6612) | 16 / 17 / 18 | 13 / 14 / 27 |
| SLF3S flow sensor (0x08) + MPRLS pressure sensor (0x18), I²C | **bus 0**: SDA 21, SCL 22 | **bus 1**: SDA 32, SCL 33 |
| Status LED (on board) | GPIO 2: slow blink idle, fast blink running, solid E-STOP | |
| Future analog sensors (ADC) | ADC1 only: GPIO 34-39 (32/33 are taken by I²C bus 1) | |

The UI rejects these pins:

* **6-11**: connected to the SPI flash, and using them crashes the chip
* **0, 2, 5, 12, 15**: strapping pins that can stop the board booting (GPIO 12 high at boot selects 1.8 V flash)
* **1, 3**: UART0, the USB serial link
* **34-39**: input-only, so they can't drive a motor or I²C
* **20, 24, 28-31**: don't exist on the ESP32
* any pin used twice

ADC2 (GPIO 0, 2, 4, 12-15, 25-27) cannot be read while Wi-Fi is on, so
analog sensors must use ADC1.

**Why two I²C buses:** every SLF3S has the fixed address **0x08**, so two
sensors on one bus collide. The ESP32 has two I²C controllers, so each channel
gets its own bus. With more than two sensors, put a **TCA9548A** I²C
multiplexer (address 0x70) on one bus and give each sensor its own mux
channel. That needs a small firmware change: select the mux channel before
each read.

## Power, read this first

```
  Motor supply (5 V for 28BYJ-48; 8-35 V for NEMA/DC) ───────┐
                                                             ├── driver VM / VMOT / + terminal
  USB 5 V ─── ESP32 DevKit ── 3V3 pin ── SLF3S VDD (3.3 V)   │
                     │                                       │
  GND ───────────────┴──────────── driver GND ───────────────┴── supply GND   (ONE common ground)
```

* Power motors from their **own supply**, not from the DevKit's 5 V/VIN pin. A
  stalled stepper or DC motor can draw amps and brown out the ESP32 (it reboots,
  and the run stops; runs never auto-resume).
* **Common ground**: ESP32 GND, every driver's logic GND and the motor
  supply GND must be connected.
* Add a **100 µF electrolytic** across VMOT/GND right at each A4988, DRV8825 or TMC2209
  (these drivers die from supply spikes otherwise).
* The SLF3S runs on **3.2-3.8 V**. Power it from the ESP32 **3V3** pin and
  **never from 5 V**.

## ULN2003 + 28BYJ-48 (unipolar stepper)

```
 ESP32           ULN2003 board                28BYJ-48 (5-wire plug)
 GPIO16 ───────► IN1                          ┌──────────────┐
 GPIO17 ───────► IN2        OUT1..OUT4 ─────► │ coils A,B,C,D │
 GPIO18 ───────► IN3        (white 5-pin      │ red = +5 V    │
 GPIO19 ───────► IN4         connector)       └──────────────┘
 GND ──────────► GND (−)
 5 V supply ───► + (5-12 V; 5 V for the 5 V motor)
```

The board's LEDs blink with the phases. The firmware drives the 8-step
half-step sequence (4096 half-steps per output-shaft revolution nominal) and
can de-energise all coils when stopped (profile option, on by default) so the
motor doesn't heat. Practical maximum is about 15 RPM.

**A ULN2003 cannot drive a bipolar (4-wire) NEMA motor.**

## STEP/DIR driver (A4988 / DRV8825 / TMC2209) + NEMA 17

```
 ESP32           A4988 / DRV8825                    NEMA 17 (bipolar, 4 wires)
 GPIO16 ───────► STEP                 1A ─┐
 GPIO17 ───────► DIR                  1B ─┴─ coil A
 GPIO18 ───────► EN  (active LOW)     2A ─┐
 3V3 ──────────► VDD (logic)          2B ─┴─ coil B
                 RST ── SLP (tie together on A4988/DRV8825)
                 MS1/MS2/MS3 ─ set microstepping (e.g. all HIGH = 1/16 on A4988)
 Motor supply ─► VMOT (+100 µF cap)   GND ── common ground
```

* Set **microstep** in the pump profile to match the MS pins (TMC2209: the
  MS1/MS2 pins or UART config).
* Set the driver's current limit (Vref) to the motor's rating **before**
  running.
* EN is optional (set it to −1 if it isn't wired; most boards pull EN low, which
  means always enabled). The polarity is selectable (*EN active-low*). With EN
  wired, E-STOP disables the driver outputs.
* The step generator is a hardware-timer interrupt (10 MHz timebase). Step
  rates up to 20 kHz are supported with about 1 µs timing jitter.

## DRV8871 (brushed DC, IN1/IN2 PWM)

```
 ESP32           DRV8871 breakout          DC pump
 GPIO16 ───────► IN1          OUT1 ───────► motor +
 GPIO17 ───────► IN2          OUT2 ───────► motor −
 GND ──────────► GND
 Motor supply ─► VM (6.5-45 V)    (set current limit with the ILIM resistor)
```

PWM is 20 kHz (inaudible) at 10 bits. Truth table (TI datasheet): IN1/IN2 =
0/0 coast, 1/0 forward, 0/1 reverse, 1/1 brake. *Fast decay* puts PWM on one input
with the other low; *slow decay* holds one input high and puts inverted PWM on
the other. Select the mode per pump profile.

## Generic PWM + DIR H-bridge (L298N, TB6612FNG)

```
 ESP32 GPIO16 ─► PWM (L298N ENA / TB6612 PWMA)
 ESP32 GPIO17 ─► DIR (L298N IN1, with IN2 = inverted DIR; TB6612 AIN1/AIN2 via inverter or use DRV8871 mode)
 ESP32 GPIO18 ─► EN/STBY (optional; TB6612 STBY)
```

L298N boards that need both IN1 and IN2 can instead use the **DRV8871** mode
(IN1 = PWM forward, IN2 = PWM reverse) with ENA jumpered high.

## Sensirion SLF3S-1300F flow sensor

6-pin connector (mating: Molex 51021-0600). Pin assignment per the
datasheet (check your cable's colour code against it):

| Pin | Signal | ESP32 (Ch 1 / Ch 2) |
|---|---|---|
| 1 | n.c. (leave floating or GND) | |
| 2 | SDA | GPIO 21 / GPIO 32 |
| 3 | VDD (3.2-3.8 V) | **3V3** |
| 4 | GND | GND |
| 5 | SCL | GPIO 22 / GPIO 33 |
| 6 | n.c. | |

```
           3V3
            │
          4.7 kΩ   4.7 kΩ          (pull-ups to 3.3 V if your breakout/cable has none;
            │        │              the ESP32's internal ~45 kΩ pull-ups are too weak at 400 kHz)
 GPIO21 ────┴────────┼──────── SLF3S pin 2 (SDA)
 GPIO22 ─────────────┴──────── SLF3S pin 5 (SCL)
 3V3 ───────────────────────── SLF3S pin 3 (VDD)
 GND ───────────────────────── SLF3S pin 4 (GND)
```

Keep I²C wires short (< 30 cm at 400 kHz). The firmware runs the bus at 400 kHz
and polls at 50 Hz. Fluidics: 2 mm ID tube, 1/4"-28 UNF flat-bottom ports.
Mount the sensor so bubbles can't get trapped (flow upward or horizontal).

## SparkFun Qwiic MicroPressure (Honeywell MPRLS0025PA00001A)

Absolute pressure, 0-25 psi (0-172 kPa), I²C address **0x18**, 3.3 V only.
It shares the channel's I²C bus with the SLF3S (0x08), so no extra pins are
needed. Enable it per channel in **Settings → Hardware → Pressure sensor**
(Ch 1 is on by default).

| Qwiic cable | Board pad | ESP32 (Ch 1 / Ch 2) |
|---|---|---|
| black | GND | GND |
| red | 3V3 | **3V3** (abs. max 3.6 V: never 5 V; the board has no regulator) |
| blue | SDA | GPIO 21 / GPIO 32 (same wire as the SLF3S SDA) |
| yellow | SCL | GPIO 22 / GPIO 33 (same wire as the SLF3S SCL) |
| — | RESET | leave unconnected (2.2 kΩ pull-up on the board) |
| — | EOC | leave unconnected (the firmware times the conversion instead) |

```
 ESP32 3V3 ──┬──────────── SLF3S pin 3 (VDD)
             └──────────── Qwiic red (3V3)
 GPIO21 ─────┬──────────── SLF3S pin 2 (SDA)
             └──────────── Qwiic blue (SDA)
 GPIO22 ─────┬──────────── SLF3S pin 5 (SCL)
             └──────────── Qwiic yellow (SCL)
 GND ────────┴─────┬────── SLF3S pin 4, Qwiic black
```

* The board has **I²C pull-up resistors** (enabled by its jumper). They serve
  the SLF3S too, so you don't need separate 4.7 kΩ resistors on that bus. If you
  already fitted them, it's still fine (≈1.5 kΩ combined).
* Use a Qwiic-to-female-jumper cable (or solder to the pads) and splice it onto
  the SLF3S's SDA/SCL/3V3/GND. The second Qwiic connector can daisy-chain.
* One MPRLS per bus: the address is fixed, so a second one on the same bus would
  collide. Ch 2's sensor goes on bus 1.
* **Plumbing:** the MPRLS has a stainless long port with **silicone gel**
  (Honeywell's liquid-media option; wetted materials: 304 stainless, gel), so
  water or saline in the line is fine. Couple the port to the line with a short
  tee and a soft tubing sleeve, keep it bubble-free, and mount it at the height
  where you want pressure referenced (1 cm of water height ≈ 0.1 kPa).
* Overpressure 60 psi, burst 120 psi (datasheet). The firmware reports
  **"over range"** above 25 psi absolute (≈ 10 psi / 70 kPa gauge at sea level).

## Bring-up checklist

1. Flash the **sim** build first and click through the UI.
2. Flash the real build. Open the serial monitor: you should see the AP name and
   `Boot complete`.
3. Settings → Hardware: confirm the drivers and pins match your wiring. Apply.
4. Dashboard: *Sensor & zero* should show **Sensor OK** (and **Pressure OK**)
   with 0 errors. With the pump stopped and the pressure port open to air,
   press **Tare pressure**.
5. Manual mode at a low command. Check the direction (use *Invert direction* in
   the profile if needed).
6. Prime, tare, calibrate ([CALIBRATION.md](CALIBRATION.md)).
