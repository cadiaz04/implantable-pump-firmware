#pragma once
// =============================================================================
// RunManager — per-channel run state machine and mode logic.
//
//   IDLE → RUNNING ⇄ PAUSED → IDLE,  plus CALIBRATING, FAULT, ESTOP
//
// Modes: MANUAL (RPM / duty %), CLOSED_LOOP (flow setpoint), OPEN_LOOP (flow
// via inverse calibration, no feedback), VOLUME ("deliver V over T"),
// PROGRAM (list of timed steps), CALIBRATION (sweep).
//
// Decides the motor command each tick from the run request, the measured
// flow, the watchdog and the safety rules. Hardware-free: the Channel glue
// (src/app/channel.cpp) feeds it sensor data and applies its output to the
// motor driver. Unit tested natively.
//
// Safety rules implemented here:
//   * flow setpoints outside [MIN_FLOW, MAX_FLOW] (config.h) are rejected
//   * measured flow > MAX_FLOW × FLOW_CEILING_TRIP_FACTOR for
//     FLOW_CEILING_TRIP_S trips to FAULT in every motor-active state
//   * heartbeat loss stops the run unless keepRunning was requested
//   * sensor FAULT / air-in-line pause (or stop) feedback-controlled runs
//   * gauge pressure above the channel's alarm limit (occlusion), pressure-
//     sensor saturation or fault pause (or stop) runs in EVERY mode and abort
//     calibration — only when a limit is set (pressureAlarmKpa > 0)
//   * E-STOP latches until resetEstop()
//   * nothing auto-resumes
// =============================================================================
#include <cstdint>
#include "calibration/calibration_sweep.h"
#include "control/flow_controller.h"
#include "control/flow_model.h"
#include "util/totalizer.h"

enum class ChannelState : uint8_t { IDLE, RUNNING, PAUSED, CALIBRATING, FAULT, ESTOP };
enum class RunMode : uint8_t { NONE, MANUAL, CLOSED_LOOP, OPEN_LOOP, VOLUME, PROGRAM, CALIBRATION };
const char* channelStateName(ChannelState s);
const char* runModeName(RunMode m);
bool runModeFromName(const char* s, RunMode& out);

enum Alarm : uint16_t {
    ALARM_AIR = 1 << 0,            // air-in-line (sensor flag, debounced)
    ALARM_SENSOR_FAULT = 1 << 1,
    ALARM_HIGH_FLOW = 1 << 2,      // sensor "high flow" flag
    ALARM_FLOW_CEILING = 1 << 3,   // firmware MAX_FLOW ceiling tripped
    ALARM_WATCHDOG = 1 << 4,       // heartbeat lost → stopped
    ALARM_ESTOP = 1 << 5,
    ALARM_NO_SENSOR = 1 << 6,      // feedback run without a usable sensor
    ALARM_OVERPRESSURE = 1 << 7,   // gauge pressure above the alarm limit (occlusion)
    ALARM_PRESSURE_SENSOR = 1 << 8,// pressure sensor fault
};

enum class StepKind : uint8_t { MANUAL, FLOW, FLOW_OPEN_LOOP };
constexpr int MAX_PROGRAM_STEPS = 16;

struct ProgramStep {
    StepKind kind = StepKind::MANUAL;
    float value = 0;        // command (MANUAL) or µL/min (FLOW*)
    float durationS = 0;
};

struct Program {
    ProgramStep steps[MAX_PROGRAM_STEPS];
    int n = 0;
    bool loop = false;
};

struct RunRequest {
    RunMode mode = RunMode::MANUAL;
    float command = 0;              // MANUAL
    bool reverse = false;
    float flowUlMin = 0;            // CLOSED_LOOP / OPEN_LOOP
    float volumeUl = 0;             // VOLUME
    float durationS = 0;            // VOLUME
    bool openLoop = false;          // VOLUME: open-loop instead of closed-loop
    float initialCommand = 0;       // closed loop without calibration (0 = none)
    bool keepRunning = false;       // ignore heartbeat loss
    Program program;                // PROGRAM
};

struct RunConfig {
    float minCommand = 0;
    float maxCommand = 0;
    FlowControlConfig flow;
    float theoreticalGain = 0;      // µL/min per command unit (0 = unknown)
    bool sensorFaultStops = false;  // false → pause, true → stop
    float airDebounceS = 0.5f;
    float volumeTailS = 15.0f;      // keep totalizing this long after the motor stops
    float pressureAlarmKpa = 0;     // gauge kPa; 0 = pressure protection off
    float pressureDebounceS = 1.0f;
};

struct ChannelInputs {
    float nowS = 0;
    float dtS = 0;
    bool sensorPresent = false;     // a flow sensor is configured on this channel
    bool sensorOk = false;          // and is currently delivering valid data
    bool newSample = false;         // a fresh sample arrived this tick
    float flowRaw = 0;              // µL/min, as measured
    float flow = 0;                 // µL/min, tare-corrected
    float flowAvg = 0;              // display / ceiling average (filter window)
    float flowCtl = 0;              // control-filtered flow
    float tempC = 0;
    bool air = false;
    bool highFlow = false;
    bool heartbeatExpired = false;
    float actualCommand = 0;        // motor's current (ramped) command
    bool pressurePresent = false;   // a pressure sensor is configured
    bool pressureOk = false;        // and delivering valid frames
    bool pressureSaturated = false; // last frame out of the sensor's range
    float pressureKpa = NAN;        // gauge pressure (tare-corrected, short average)
};

struct ChannelOutputs {
    float command = 0;
    bool reverse = false;
    bool active = false;            // motor should be energised
};

struct RunEvent {
    uint8_t level = 0;              // 0 info, 1 warn, 2 alarm
    char msg[80] = {};
};

class RunManager {
public:
    void configure(const RunConfig& cfg);
    const RunConfig& config() const { return cfg_; }
    // Active calibration model (nullptr if none). Non-owning.
    void setCalibrationModel(const FlowModel* m) { calModel_ = m; }
    bool hasCalibration() const { return calModel_ != nullptr; }

    // ---- commands (return nullptr on success, else a reason) ----
    const char* start(const RunRequest& req, float nowS, bool sensorPresent, bool sensorOk);
    const char* startCalibration(const SweepConfig& sweep, bool keepRunning, float nowS);
    // Re-measure one point of a finished/aborted sweep (keeps the other data).
    const char* retryCalibrationPoint(int index, bool keepRunning, float nowS);
    const char* pause(float nowS);
    const char* resume(float nowS, bool sensorOk, bool air);
    void stop(float nowS, const char* reason = nullptr);
    void estop();
    void resetEstop();
    const char* startZero(float seconds, float nowS);
    void resetVolume();

    ChannelOutputs tick(const ChannelInputs& in);

    // ---- state ----
    ChannelState state() const { return state_; }
    RunMode mode() const { return mode_; }
    const RunRequest& request() const { return req_; }
    bool keepRunning() const { return req_.keepRunning; }
    uint16_t alarms() const { return alarms_; }
    float targetCommand() const { return out_.command; }
    float flowSetpoint() const;            // µL/min, NAN if not a flow mode
    double volumeUl() const { return total_.volumeUl(); }
    bool volumeEstimated() const { return volumeEstimated_; }
    double runVolumeUl() const { return runTotal_.volumeUl(); }
    float plannedVolumeUl() const;
    float etaS() const;                    // NAN if not applicable
    float runElapsedS() const { return runElapsedS_; }
    int programStep() const { return stepIdx_; }
    float stepElapsedS() const { return stepElapsedS_; }
    const FlowController& flowController() const { return fc_; }
    CalibrationSweep& sweep() { return sweep_; }
    const CalibrationSweep& sweep() const { return sweep_; }
    bool zeroing() const { return zeroing_; }
    float zeroProgress(float nowS) const;
    // One-shot result of a finished tare (offset = mean raw flow).
    bool takeZeroResult(float& offset, bool& ok);
    bool usesFeedback() const;
    bool overpressure() const { return overpressure_; }

    bool popEvent(RunEvent& e);
    // Lets the channel glue report its own events through the same queue.
    void note(const char* msg, uint8_t level = 0) { emit(level, "%s", msg); }

private:
    void emit(uint8_t level, const char* fmt, ...);
    const char* validateFlow(float f) const;
    const FlowModel* openLoopModel() const;
    float estimateFlow(float command) const;
    void enterStep(int i, float nowS);
    void endRun(const char* msg, uint8_t level);
    float runMode(const ChannelInputs& in);

    RunConfig cfg_;
    const FlowModel* calModel_ = nullptr;
    ProportionalFlowModel fallbackModel_;
    ProportionalFlowModel theoryModel_;
    FlowController fc_;
    CalibrationSweep sweep_;

    ChannelState state_ = ChannelState::IDLE;
    RunMode mode_ = RunMode::NONE;
    RunRequest req_;
    ChannelOutputs out_;
    uint16_t alarms_ = 0;
    uint16_t latched_ = 0;

    float runElapsedS_ = 0;
    int stepIdx_ = -1;
    float stepElapsedS_ = 0;
    float stepCommand_ = 0;
    float airS_ = 0;
    float ceilingS_ = 0;
    float sinceActiveS_ = 1e9f;
    float overS_ = 0;
    bool overpressure_ = false;

    Totalizer total_;
    Totalizer runTotal_;
    bool volumeEstimated_ = false;

    bool zeroing_ = false;
    float zeroStart_ = 0, zeroLen_ = 0;
    double zeroSum_ = 0;
    uint32_t zeroN_ = 0;
    bool zeroAir_ = false;
    bool zeroReady_ = false, zeroOk_ = false;
    float zeroOffset_ = 0;

    static constexpr int kEvents = 8;
    RunEvent events_[kEvents];
    int evHead_ = 0, evCount_ = 0;
};
