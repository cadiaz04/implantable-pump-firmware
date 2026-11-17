#include "pump_profile.h"

PumpProfile starterProfile28byj() {
    PumpProfile p;
    p.id = "28byj48-syringe";
    p.name = "28BYJ-48 Syringe pump (example)";
    p.pumpType = "syringe";
    p.notes = "Template: 28BYJ-48 + ULN2003 driving a 1 mL syringe via M5 lead screw. Edit geometry, then calibrate.";
    p.motor.driverType = DriverType::ULN2003;
    p.motor.stepsPerRev = 4096;   // half-steps per output rev (real units ≈ 4075.77)
    p.motor.microstep = 1;
    p.motor.maxCommand = 15;      // RPM — 28BYJ-48 stalls above ~15-20 RPM
    p.motor.minCommand = 0;
    p.motor.accel = 10;
    p.motor.releaseWhenStopped = true;
    p.syringeIdMm = 4.78f;        // BD 1 mL
    p.leadscrewPitchMm = 0.8f;    // M5 coarse
    p.pid.filterWindowS = 10;
    return p;
}

PumpProfile starterProfileNema17() {
    PumpProfile p;
    p.id = "nema17-syringe";
    p.name = "NEMA17 Syringe pump (example)";
    p.pumpType = "syringe";
    p.notes = "Template: NEMA17 on A4988/DRV8825/TMC2209 at 1/16 microstepping, 10 mL syringe, 2 mm lead screw.";
    p.motor.driverType = DriverType::STEP_DIR;
    p.motor.stepsPerRev = 200;
    p.motor.microstep = 16;
    p.motor.maxCommand = 2.5f;    // RPM — keeps theoretical flow under the firmware ceiling
    p.motor.minCommand = 0;
    p.motor.accel = 5;
    p.motor.releaseWhenStopped = false;
    p.syringeIdMm = 14.5f;        // BD 10 mL
    p.leadscrewPitchMm = 2.0f;
    p.pid.filterWindowS = 10;
    return p;
}

PumpProfile starterProfileDcDiaphragm() {
    PumpProfile p;
    p.id = "dc-diaphragm";
    p.name = "DC Diaphragm pump (example)";
    p.pumpType = "diaphragm";
    p.notes = "Template: brushed DC diaphragm pump on a DRV8871. Pulsatile: use an averaging window of several pump cycles.";
    p.motor.driverType = DriverType::DRV8871;
    p.motor.maxCommand = 100;     // duty %
    p.motor.minCommand = 15;
    p.motor.minDuty = 12;
    p.motor.kickMs = 150;
    p.motor.kickDuty = 60;
    p.motor.accel = 50;
    p.motor.decay = DecayMode::FAST;
    p.pid.filterWindowS = 20;
    return p;
}
