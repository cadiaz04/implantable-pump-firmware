#pragma once
// =============================================================================
// Channel — one pump channel: glues the pure RunManager to a MotorDriver, a
// FlowSensor and an optional PressureSensor (both on the channel's own I²C
// bus, which the channel owns). Owns the flow tare offset, the pressure tare
// (atmosphere) offset, rolling statistics and the active calibration model.
//
// Threading: pollSensor()/tick()/applyHardware() run in the real-time task
// (core 1). Everything else is called with the app mutex held (any task).
// =============================================================================
#include <ArduinoJson.h>
#include "config/hardware_config.h"
#include "control/flow_model.h"
#include "control/run_manager.h"
#include "motors/motor_driver.h"
#include "pumps/pump_profile.h"
#include "sensors/sensor.h"
#include "util/window_stats.h"
#ifdef SIM_MODE
#include "sensors/sim_plant.h"
#endif

struct ChannelSnapshot {
    ChannelState state = ChannelState::IDLE;
    RunMode mode = RunMode::NONE;
    DriverType driver = DriverType::NONE;
    float command = 0;          // target command
    float commandActual = 0;    // ramped command at the motor
    float stepRateHz = 0;
    float setpoint = NAN;       // µL/min (flow modes)
    float manualCommand = NAN;  // manual mode command
    bool reverse = false;
    float flow = NAN, flowAvg = NAN, flowStd = NAN, tempC = NAN;
    uint16_t flags = 0;
    uint16_t alarms = 0;
    double volumeUl = 0;
    bool volumeEstimated = false;
    double runVolumeUl = 0;
    float plannedVolumeUl = NAN, targetVolumeUl = NAN, etaS = NAN, runElapsedS = 0;
    SensorStatus sensorStatus = SensorStatus::ABSENT;
    SensorErrors errors;
    float zeroOffset = 0;
    bool zeroing = false;
    float zeroProgress = 0;
    bool keepRunning = false;
    int programStep = -1;
    bool hasProfile = false;
    bool profileMismatch = false;
    char profileId[41] = "";
    char profileName[64] = "";
    bool hasCalibration = false;
    float ff = NAN, corr = NAN;
    SweepPhase calPhase = SweepPhase::IDLE;
    int calIndex = -1, calPoints = 0;
    float calPhaseElapsed = 0, calPhaseDuration = 0;
    float filterWindowS = 10;
    // pressure (gauge = absolute − tare)
    bool pressurePresent = false;
    SensorStatus pressureStatus = SensorStatus::ABSENT;
    SensorErrors pressureErrors;
    float pressureKpa = NAN;        // gauge, 1-s average
    float pressureAbsKpa = NAN;     // latest absolute
    float pressureZeroKpa = NAN;    // tare (NAN = never tared → standard atmosphere used)
    bool pressureSaturated = false;
    uint8_t pressureStatusByte = 0;
    bool pressureTaring = false;
    float pressureTareProgress = 0;
    float pressureAlarmKpa = 0;     // 0 = off
};

struct HistoryChannel {
    float flow = NAN;
    float setpoint = NAN;
    float command = 0;
    int16_t tempCc = 0;       // centi-°C
    int16_t pressureCkPa = 0; // gauge, 0.01 kPa (INT16_MIN = none)
    uint8_t state = 0;
    uint8_t flags = 0;
};

class Channel {
public:
    void init(int idx);
    int index() const { return idx_; }

    // ---- real-time task ----
    void applyHardware(const ChannelHw& hw);
    void pollSensor(uint32_t nowMs);
    void tick(uint32_t nowMs, float dtS, bool hbExpired);
    void takeHistory(HistoryChannel& h);

    // ---- with mutex held ----
    void setProfile(const PumpProfile& p);
    bool hasProfile() const { return hasProfile_; }
    bool profileMatchesDriver() const;
    const PumpProfile& profile() const { return profile_; }
    const ChannelHw& hardware() const { return hw_; }
    bool isIdle() const;

    const char* run(const RunRequest& r, uint32_t nowMs);
    void stop(uint32_t nowMs);
    const char* pause(uint32_t nowMs);
    const char* resume(uint32_t nowMs);
    void estop();
    void resetEstop();
    const char* zero(float seconds, uint32_t nowMs);
    void setZeroOffset(float v) { zeroOffset_ = v; }
    float zeroOffset() const { return zeroOffset_; }
    void resetVolume() { rm_.resetVolume(); }

    const char* calStart(const SweepConfig& cfg, bool keepRunning, uint32_t nowMs);
    void calAbort(uint32_t nowMs);
    bool calReference(int index, float flowUlMin, bool clear);
    const char* calRetry(int index, bool keepRunning, uint32_t nowMs);
    void calDiscard();
    // Builds a calibration from the sweep, attaches it to the profile (active)
    // and returns it. Caller persists the profile.
    bool calAccept(FitType type, const String& name, const String& note, uint32_t epochS, String& err);
    void calStatusJson(JsonObject o, uint32_t nowMs) const;

    void snapshot(ChannelSnapshot& s, uint32_t nowMs) const;
    bool popEvent(RunEvent& e) { return rm_.popEvent(e); }
    bool takeZeroSave(float& v);

    // ---- pressure ----
    bool hasPressure() const { return psensor_ != nullptr; }
    void setPressureZero(float absKpa) { pZero_ = absKpa; }
    float pressureZero() const { return pZero_; }
    const char* pressureTare(float seconds, uint32_t nowMs);
    void setPressureAlarm(float gaugeKpa);
    float pressureAlarm() const { return pAlarm_; }
    bool takePressureZeroSave(float& v);

    MotorDriver* motor() { return motor_; }
    FlowSensor* sensor() { return sensor_; }
#ifdef SIM_MODE
    SimPlant& plant() { return plant_; }
#endif

private:
    void rebuildRunConfig();
    void configureSim();

    int idx_ = 0;
    ChannelHw hw_;
    PumpProfile profile_;
    bool hasProfile_ = false;
    MotorDriver* motor_ = nullptr;
    FlowSensor* sensor_ = nullptr;
    PressureSensor* psensor_ = nullptr;
    bool busBegun_ = false;
#ifdef SIM_MODE
    SimPlant plant_;
    uint32_t simLastMs_ = 0;
#endif
    RunManager rm_;
    CurveFlowModel calModel_;
    bool calUsable_ = false;
    bool estop_ = false;
    bool settingsSensorFaultStops_ = false;

    WindowStats avg_{10.0f};
    Ema ema_{5.0f};
    bool newSample_ = false;
    float lastSampleS_ = 0;
    float flow_ = NAN;
    float zeroOffset_ = 0;
    bool zeroDirty_ = false;

    // pressure
    bool pNew_ = false;
    WindowStats pAvg_{1.0f};
    float pZero_ = NAN;
    bool pZeroDirty_ = false;
    float pAlarm_ = 0;
    bool pTaring_ = false;
    float pTareStart_ = 0, pTareLen_ = 0;
    double pTareSum_ = 0;
    uint32_t pTareN_ = 0;
    float gaugeKpa() const;

    // 1-s history accumulators
    double histSum_ = 0;
    uint32_t histN_ = 0;
    double pHistSum_ = 0;
    uint32_t pHistN_ = 0;

public:
    void setSensorFaultStops(bool v) {
        settingsSensorFaultStops_ = v;
        rebuildRunConfig();
    }
};
