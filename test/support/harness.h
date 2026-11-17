#pragma once
// Shared test harness: RunManager + SimPlant wired like src/app/channel.cpp
// (100 Hz tick, 50 Hz sensor, motor ramp, tare offset, rolling averages).
#include "control/run_manager.h"
#include "motors/motor_types.h"
#include "sensors/sim_plant.h"
#include "util/window_stats.h"

struct Harness {
    RunManager rm;
    SimPlant plant;
    WindowStats avg{10.0f};
    float t = 0;
    float zeroOffset = 0;
    float actual = 0;
    float accel = 1000;
    bool sensorPresent = true;
    bool sensorOk = true;
    bool hbExpired = false;
    bool forceAir = false;
    bool pressurePresent = false;
    bool pressureOk = true;
    float pressureZero = 100.8f;   // SimPlant default atmosphere
    long ticks = 0;
    ChannelOutputs out;

    explicit Harness(float maxCommand = 15, float minCommand = 0) {
        RunConfig c;
        c.maxCommand = maxCommand;
        c.minCommand = minCommand;
        rm.configure(c);
        SimPlantParams p;
        p.gain = 20;
        p.zeroOffsetUlMin = 8;
        p.noiseUlMin = 5;
        p.tauS = 4;
        p.deadTimeS = 2;
        plant.setParams(p);
    }

    void run(float seconds) {
        const int n = static_cast<int>(seconds / 0.01f + 0.5f);
        for (int i = 0; i < n; ++i) tick();
    }

    void tick() {
        const float dt = 0.01f;
        t += dt;
        ++ticks;
        plant.setCommand(actual, out.reverse);
        ChannelInputs in;
        in.nowS = t;
        in.dtS = dt;
        in.sensorPresent = sensorPresent;
        in.sensorOk = sensorOk && sensorPresent;
        in.newSample = in.sensorOk && (ticks % 2 == 0);
        if (in.newSample) {
            in.flowRaw = plant.step(0.02f);
            in.flow = in.flowRaw - zeroOffset;
            avg.add(t, in.flow);
        }
        in.flowAvg = avg.mean();
        in.flowCtl = avg.mean();
        in.tempC = 23;
        in.air = forceAir || plant.bubbleActive();
        in.heartbeatExpired = hbExpired;
        in.actualCommand = actual;
        in.pressurePresent = pressurePresent;
        in.pressureOk = pressurePresent && pressureOk;
        if (in.pressureOk) {
            in.pressureSaturated = plant.pressureAbsKpa() > SimPlant::kPressureFullScaleKpa;
            in.pressureKpa = plant.pressureAbsKpa() - pressureZero;
        }
        out = rm.tick(in);
        actual = rampToward(actual, out.command, accel, dt);
        float off;
        bool ok;
        if (rm.takeZeroResult(off, ok) && ok) zeroOffset = off;
        if (rm.sweep().takeZeroResult(off, ok) && ok) zeroOffset = off;
    }
};
