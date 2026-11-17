#include "channel.h"
#include <Arduino.h>
#include <Wire.h>
#include "motors/motor_factory.h"
#include "pumps/profile_json.h"
#include "config.h"
#include "sensors/mprls_protocol.h"
#ifdef SIM_MODE
#include "sensors/sim_flow_sensor.h"
#include "sensors/sim_pressure_sensor.h"
#else
#include "sensors/mprls_sensor.h"
#include "sensors/slf3s_sensor.h"
#endif

void Channel::init(int idx) {
    idx_ = idx;
#ifdef SIM_MODE
    plant_ = SimPlant(0xC0FFEEu + 7919u * idx);
#endif
}

bool Channel::isIdle() const {
    const ChannelState s = rm_.state();
    return (s == ChannelState::IDLE || s == ChannelState::ESTOP || s == ChannelState::FAULT) && !rm_.zeroing();
}

bool Channel::profileMatchesDriver() const {
    return hasProfile_ && familyOf(profile_.motor.driverType) == familyOf(hw_.driver);
}

void Channel::configureSim() {
#ifdef SIM_MODE
    SimPlantParams p;
    if (familyOf(hw_.driver) == DriverFamily::DC) {
        p.gain = 6;
        p.quad = 0.02f;
        p.deadband = 12;
        p.tauS = 3;
        p.deadTimeS = 2;
        p.noiseUlMin = 5;
        p.zeroOffsetUlMin = -5;
        p.pulsationPct = 25;
        p.pulsationHzPerCmd = 0.04f;
    } else {
        // Behave like the profile's syringe, minus 8 % (seal friction, slip)
        // so calibration finds something different from theory.
        const float th = hasProfile_ ? profile_.theoreticalGain() : 0;
        p.gain = th > 0 ? 0.92f * th : 20;
        p.quad = -0.002f * p.gain;
        p.tauS = 4;
        p.deadTimeS = 2;
        p.noiseUlMin = 5;
        p.zeroOffsetUlMin = 8 + 3 * idx_;
    }
    plant_.setParams(p);
#endif
}

void Channel::rebuildRunConfig() {
    RunConfig c;
    const bool ok = profileMatchesDriver();
    if (ok) {
        c.minCommand = profile_.motor.minCommand;
        c.maxCommand = profile_.motor.maxCommand;
        c.theoreticalGain = profile_.theoreticalGain();
    }
    const PidParams& q = profile_.pid;
    c.flow.kp = q.kp;
    c.flow.ki = q.ki;
    c.flow.kd = q.kd;
    c.flow.loopHz = q.loopHz;
    c.flow.outputLimitPct = q.outputLimitPct;
    c.flow.integralLimitPct = q.integralLimitPct;
    c.sensorFaultStops = settingsSensorFaultStops_;
    c.pressureAlarmKpa = pAlarm_;
    rm_.configure(c);

    if (std::fabs(avg_.windowS() - q.filterWindowS) > 1e-3f) avg_.configure(q.filterWindowS);
    ema_.setTau(q.filterWindowS * 0.5f);

    if (motor_) {
        MotorParams mp = profile_.motor;
        mp.driverType = hw_.driver;
        if (!ok) mp.maxCommand = 0;   // mismatched profile: driver refuses to move
        motor_->setParams(mp);
    }

    calUsable_ = false;
    if (ok) {
        if (const Calibration* cal = profile_.activeCalibration()) {
            calModel_.set(cal->fit);
            calUsable_ = cal->fit.usableIn(c.minCommand, c.maxCommand);
        }
    }
    rm_.setCalibrationModel(calUsable_ ? &calModel_ : nullptr);
}

void Channel::setProfile(const PumpProfile& p) {
    profile_ = p;
    hasProfile_ = true;
    rebuildRunConfig();
    configureSim();
}

void Channel::applyHardware(const ChannelHw& hw) {
    rm_.stop(millis() / 1000.0f, nullptr);
    if (motor_) {
        motor_->stop();
        motor_->end();
        delete motor_;
        motor_ = nullptr;
    }
    if (sensor_) {
        delete sensor_;
        sensor_ = nullptr;
    }
    if (psensor_) {
        delete psensor_;
        psensor_ = nullptr;
    }
#ifndef SIM_MODE
    TwoWire& bus = idx_ == 0 ? Wire : Wire1;
    if (busBegun_) bus.end();
    busBegun_ = false;
#endif
    pTaring_ = false;
    hw_ = hw;
    motor_ = createMotor(hw, idx_);
    if (motor_) {
        rebuildRunConfig();   // sets motor params
        if (!motor_->begin()) Serial.printf("[ch%d] motor driver begin() failed\n", idx_ + 1);
        motor_->enable(!estop_);
    }
#ifndef SIM_MODE
    // One bus per channel, shared by the SLF3S (0x08) and the MPRLS (0x18).
    if (hw.usesBus()) {
        busBegun_ = bus.begin(hw.sda, hw.scl, 400000);
        bus.setTimeOut(10);
    }
    const bool shared = hw.sensor != SensorType::NONE && hw.pressure != PressureType::NONE;
#endif
    if (hw.sensor == SensorType::SLF3S) {
#ifdef SIM_MODE
        sensor_ = new SimFlowSensor(plant_, idx_);
#else
        sensor_ = new Slf3sSensor(bus, hw.sda, hw.scl, idx_, shared);
#endif
        sensor_->begin();
    }
    if (hw.pressure == PressureType::MPRLS) {
#ifdef SIM_MODE
        psensor_ = new SimPressureSensor(plant_, idx_);
#else
        psensor_ = new MprlsSensor(bus, idx_);
#endif
        psensor_->begin();
    }
    pAvg_.reset();
    avg_.reset();
    ema_.reset();
    flow_ = NAN;
    configureSim();
    rebuildRunConfig();
}

void Channel::pollSensor(uint32_t nowMs) {
#ifdef SIM_MODE
    // One plant step serves both simulated sensors.
    if (simLastMs_ == 0) simLastMs_ = nowMs;
    if (nowMs - simLastMs_ >= SENSOR_POLL_PERIOD_MS) {
        plant_.step((nowMs - simLastMs_) / 1000.0f);
        simLastMs_ = nowMs;
    }
#endif
    if (sensor_ && sensor_->poll(nowMs)) newSample_ = true;
    if (psensor_ && psensor_->poll(nowMs)) pNew_ = true;
}

float Channel::gaugeKpa() const {
    return pAvg_.mean() - (std::isfinite(pZero_) ? pZero_ : mprls::STD_ATMOSPHERE_KPA);
}

const char* Channel::pressureTare(float seconds, uint32_t nowMs) {
    if (!psensor_) return "no pressure sensor on this channel";
    if (psensor_->status() != SensorStatus::OK) return "pressure sensor is not OK";
    const ChannelState st = rm_.state();
    if (st == ChannelState::RUNNING || st == ChannelState::CALIBRATING) return "stop the pump first (tare with the line open to air)";
    if (!(seconds >= 1 && seconds <= 60)) return "tare duration must be 1-60 s";
    pTaring_ = true;
    pTareStart_ = nowMs / 1000.0f;
    pTareLen_ = seconds;
    pTareSum_ = 0;
    pTareN_ = 0;
    return nullptr;
}

void Channel::setPressureAlarm(float gaugeKpa) {
    pAlarm_ = gaugeKpa;
    rebuildRunConfig();
}

bool Channel::takePressureZeroSave(float& v) {
    if (!pZeroDirty_) return false;
    pZeroDirty_ = false;
    v = pZero_;
    return true;
}

void Channel::tick(uint32_t nowMs, float dtS, bool hbExpired) {
    const float now = nowMs / 1000.0f;
    ChannelInputs in;
    in.nowS = now;
    in.dtS = dtS;
    in.sensorPresent = sensor_ != nullptr;
    in.sensorOk = sensor_ && sensor_->status() == SensorStatus::OK;
    in.newSample = newSample_ && in.sensorOk;
    newSample_ = false;
    if (in.newSample) {
        in.flowRaw = sensor_->flowUlMin();
        in.flow = in.flowRaw - zeroOffset_;
        flow_ = in.flow;
        avg_.add(now, in.flow);
        const float sdt = lastSampleS_ > 0 ? now - lastSampleS_ : 0.02f;
        lastSampleS_ = now;
        ema_.update(in.flow, sdt);
        histSum_ += in.flow;
        ++histN_;
    }
    if (sensor_) {
        in.tempC = sensor_->tempC();
        in.air = in.sensorOk && sensor_->airInLine();
        in.highFlow = in.sensorOk && sensor_->highFlow();
    }
    in.flowAvg = in.sensorOk ? avg_.mean() : NAN;
    in.flowCtl = profile_.pid.filterEma ? ema_.value() : avg_.mean();
    in.heartbeatExpired = hbExpired;
    in.actualCommand = motor_ ? motor_->currentCommand() : 0;
    if (!in.sensorOk) {
        flow_ = NAN;
    }

    // ---- pressure ----
    if (psensor_) {
        const bool pOk = psensor_->status() == SensorStatus::OK;
        if (pNew_ && pOk && !psensor_->saturated()) {
            const float abs = psensor_->absKpa();
            pAvg_.add(now, abs);
            pHistSum_ += abs;
            ++pHistN_;
            if (pTaring_) {
                pTareSum_ += abs;
                ++pTareN_;
            }
        }
        pNew_ = false;
        if (pTaring_ && now - pTareStart_ >= pTareLen_) {
            pTaring_ = false;
            if (pTareN_ > 0) {
                pZero_ = static_cast<float>(pTareSum_ / pTareN_);
                pZeroDirty_ = true;
                char msg[80];
                snprintf(msg, sizeof(msg), "Pressure tare: atmosphere = %.2f kPa", pZero_);
                rm_.note(msg);
            }
        }
        in.pressurePresent = true;
        in.pressureOk = pOk && pAvg_.count() > 0;
        in.pressureSaturated = psensor_->saturated();
        in.pressureKpa = in.pressureOk ? gaugeKpa() : NAN;
    }

    const ChannelOutputs out = rm_.tick(in);

    float off;
    bool ok;
    if ((rm_.takeZeroResult(off, ok) || rm_.sweep().takeZeroResult(off, ok)) && ok) {
        zeroOffset_ = off;
        zeroDirty_ = true;
        avg_.reset();
        ema_.reset();
    }

    if (motor_) {
        if (estop_) {
            motor_->stop();
        } else {
            motor_->setTarget(out.active ? out.command : 0, out.reverse);
            motor_->update(dtS);
        }
#ifdef SIM_MODE
        plant_.setCommand(motor_->currentCommand(), motor_->direction());
#endif
    }
}

bool Channel::takeZeroSave(float& v) {
    if (!zeroDirty_) return false;
    zeroDirty_ = false;
    v = zeroOffset_;
    return true;
}

void Channel::takeHistory(HistoryChannel& h) {
    h.flow = histN_ ? static_cast<float>(histSum_ / histN_) : NAN;
    histSum_ = 0;
    histN_ = 0;
    h.setpoint = rm_.flowSetpoint();
    h.command = motor_ ? motor_->currentCommand() : 0;
    const float t = sensor_ ? sensor_->tempC() : NAN;
    h.tempCc = std::isfinite(t) ? static_cast<int16_t>(t * 100) : INT16_MIN;
    const float pg = pHistN_ ? static_cast<float>(pHistSum_ / pHistN_) - (std::isfinite(pZero_) ? pZero_ : mprls::STD_ATMOSPHERE_KPA) : NAN;
    pHistSum_ = 0;
    pHistN_ = 0;
    h.pressureCkPa = (std::isfinite(pg) && std::fabs(pg) < 320) ? static_cast<int16_t>(pg * 100) : INT16_MIN;
    h.state = static_cast<uint8_t>(rm_.state());
    h.flags = static_cast<uint8_t>(rm_.alarms() & 0xFF);
}

const char* Channel::run(const RunRequest& r, uint32_t nowMs) {
    if (!hasProfile_) return "no pump profile selected for this channel";
    if (!profileMatchesDriver()) return "pump profile is for a different motor type than this channel's driver";
    if (!motor_) return "no motor driver configured";
    const bool present = sensor_ != nullptr;
    const bool ok = present && sensor_->status() == SensorStatus::OK;
    return rm_.start(r, nowMs / 1000.0f, present, ok);
}

void Channel::stop(uint32_t nowMs) { rm_.stop(nowMs / 1000.0f); }
const char* Channel::pause(uint32_t nowMs) { return rm_.pause(nowMs / 1000.0f); }

const char* Channel::resume(uint32_t nowMs) {
    const bool ok = sensor_ && sensor_->status() == SensorStatus::OK;
    const bool air = ok && sensor_->airInLine();
    return rm_.resume(nowMs / 1000.0f, ok, air);
}

void Channel::estop() {
    estop_ = true;
    rm_.estop();
    if (motor_) {
        motor_->stop();
        motor_->enable(false);
    }
}

void Channel::resetEstop() {
    estop_ = false;
    rm_.resetEstop();
    if (motor_) {
        motor_->stop();
        motor_->enable(true);
    }
}

const char* Channel::zero(float seconds, uint32_t nowMs) {
    if (!sensor_) return "no flow sensor on this channel";
    if (sensor_->status() != SensorStatus::OK) return "flow sensor is not OK";
    return rm_.startZero(seconds, nowMs / 1000.0f);
}

const char* Channel::calStart(const SweepConfig& cfg, bool keepRunning, uint32_t nowMs) {
    if (!hasProfile_) return "no pump profile selected for this channel";
    if (!profileMatchesDriver()) return "pump profile is for a different motor type than this channel's driver";
    if (!sensor_) return "no flow sensor on this channel";
    if (sensor_->status() != SensorStatus::OK) return "flow sensor is not OK";
    return rm_.startCalibration(cfg, keepRunning, nowMs / 1000.0f);
}

void Channel::calAbort(uint32_t nowMs) {
    if (rm_.state() == ChannelState::CALIBRATING) rm_.stop(nowMs / 1000.0f, "Calibration aborted");
}

bool Channel::calReference(int index, float flowUlMin, bool clear) {
    return clear ? rm_.sweep().clearReference(index) : rm_.sweep().setReference(index, flowUlMin);
}

const char* Channel::calRetry(int index, bool keepRunning, uint32_t nowMs) {
    if (!sensor_ || sensor_->status() != SensorStatus::OK) return "flow sensor is not OK";
    return rm_.retryCalibrationPoint(index, keepRunning, nowMs / 1000.0f);
}

void Channel::calDiscard() {
    if (rm_.state() != ChannelState::CALIBRATING) rm_.sweep().reset();
}

bool Channel::calAccept(FitType type, const String& name, const String& note, uint32_t epochS, String& err) {
    const CalibrationSweep& sw = rm_.sweep();
    if (sw.running()) {
        err = "sweep still running";
        return false;
    }
    if (sw.pointCount() == 0) {
        err = "no calibration data";
        return false;
    }
    float x[MAX_SWEEP_POINTS], y[MAX_SWEEP_POINTS];
    const int n = sw.collect(x, y, MAX_SWEEP_POINTS);
    CurveFit fit;
    const char* e = nullptr;
    if (!fitCurve(x, y, n, type, fit, &e)) {
        err = e;
        return false;
    }
    if (!fit.usableIn(profile_.motor.minCommand, profile_.motor.maxCommand)) {
        err = "fit does not increase over the pump's command range; check the data";
        return false;
    }
    Calibration c;
    c.timestamp = epochS;
    c.id = "c" + std::to_string(epochS ? epochS : millis());
    c.name = name.length() ? name.c_str() : (std::string(fitTypeId(type)) + " fit, " + std::to_string(n) + " pts");
    c.fluid = profile_.fluid;
    c.tubingNote = note.c_str();
    c.fit = fit;
    c.active = true;
    for (int i = 0; i < sw.pointCount(); ++i) {
        const SweepPoint& p = sw.point(i);
        CalPointRec r;
        r.command = p.command;
        r.flow = p.flow;
        r.ref = p.ref;
        r.std = p.std;
        r.tempC = p.tempC;
        r.n = p.n;
        r.pass = p.pass;
        r.status = pointStatusName(p.status);
        c.points.push_back(r);
    }
    for (auto& old : profile_.calibrations) old.active = false;
    profile_.calibrations.push_back(c);
    // Keep at most 10: drop the oldest inactive ones.
    while (profile_.calibrations.size() > 10) {
        for (size_t i = 0; i < profile_.calibrations.size(); ++i) {
            if (!profile_.calibrations[i].active) {
                profile_.calibrations.erase(profile_.calibrations.begin() + i);
                break;
            }
        }
    }
    rebuildRunConfig();
    rm_.sweep().reset();
    return true;
}

void Channel::calStatusJson(JsonObject o, uint32_t nowMs) const {
    const CalibrationSweep& sw = rm_.sweep();
    const float now = nowMs / 1000.0f;
    o["phase"] = sweepPhaseName(sw.phase());
    o["running"] = sw.running();
    o["index"] = sw.currentIndex();
    o["phaseElapsed"] = sw.running() ? sw.phaseElapsedS(now) : 0;
    o["phaseDuration"] = sw.phaseDurationS();
    o["commandUnit"] = commandUnitFor(hw_.driver);
    o["zeroOffset"] = zeroOffset_;
    const SweepConfig& c = sw.config();
    JsonObject cfg = o["config"].to<JsonObject>();
    cfg["settleS"] = c.settleS;
    cfg["averageS"] = c.averageS;
    cfg["repeats"] = c.repeats;
    cfg["upDown"] = c.upDown;
    cfg["zeroFirst"] = c.zeroFirst;
    // Rough remaining-time estimate (assumes steady state is reached quickly).
    float remaining = 0;
    if (sw.running()) {
        remaining = sw.phaseDurationS() - sw.phaseElapsedS(now);
        if (sw.phase() == SweepPhase::STEADY) remaining = c.averageS;
        else if (sw.phase() == SweepPhase::SETTLE) remaining += c.averageS;
        else if (sw.phase() == SweepPhase::ZERO_SETTLE) remaining += c.zeroS + c.settleS + c.averageS;
        else if (sw.phase() == SweepPhase::ZERO) remaining += c.settleS + c.averageS;
        for (int i = sw.currentIndex() + 1; i < sw.pointCount(); ++i) {
            if (sw.point(i).status == PointStatus::PENDING) remaining += c.settleS + c.averageS;
        }
        if (remaining < 0) remaining = 0;
    }
    o["remainingS"] = remaining;
    JsonArray pts = o["points"].to<JsonArray>();
    auto num = [](JsonObject j, const char* k, float v) {
        if (std::isfinite(v)) j[k] = v;
        else j[k] = nullptr;
    };
    for (int i = 0; i < sw.pointCount(); ++i) {
        const SweepPoint& p = sw.point(i);
        JsonObject j = pts.add<JsonObject>();
        j["i"] = i;
        j["command"] = p.command;
        j["pass"] = p.pass;
        j["status"] = pointStatusName(p.status);
        num(j, "flow", p.flow);
        num(j, "std", p.std);
        num(j, "temp", p.tempC);
        num(j, "ref", p.ref);
        j["n"] = p.n;
    }
    float x[MAX_SWEEP_POINTS], y[MAX_SWEEP_POINTS];
    const int n = sw.collect(x, y, MAX_SWEEP_POINTS);
    o["validPoints"] = n;
    JsonObject fits = o["fits"].to<JsonObject>();
    const FitType types[] = {FitType::LINEAR, FitType::QUADRATIC, FitType::LOOKUP};
    for (FitType t : types) {
        CurveFit f;
        const char* err = nullptr;
        JsonObject fj = fits[fitTypeId(t)].to<JsonObject>();
        if (fitCurve(x, y, n, t, f, &err)) {
            fitToJson(f, fj);
            fj["usable"] = f.usableIn(profile_.motor.minCommand, profile_.motor.maxCommand);
            JsonArray res = fj["residuals"].to<JsonArray>();
            for (int i = 0; i < n; ++i) res.add(y[i] - f.eval(x[i]));
        } else {
            fj["error"] = err;
        }
    }
}

void Channel::snapshot(ChannelSnapshot& s, uint32_t nowMs) const {
    const float now = nowMs / 1000.0f;
    s.state = rm_.state();
    s.mode = rm_.mode();
    s.driver = hw_.driver;
    s.command = rm_.targetCommand();
    s.commandActual = motor_ ? motor_->currentCommand() : 0;
    s.stepRateHz = motor_ ? motor_->stepRateHz() : 0;
    s.setpoint = rm_.flowSetpoint();
    s.manualCommand = rm_.mode() == RunMode::MANUAL ? rm_.request().command : NAN;
    s.reverse = rm_.request().reverse;
    const bool ok = sensor_ && sensor_->status() == SensorStatus::OK;
    s.flow = ok ? flow_ : NAN;
    s.flowAvg = ok ? avg_.mean() : NAN;
    s.flowStd = ok ? avg_.stddev() : NAN;
    s.tempC = ok ? sensor_->tempC() : NAN;
    s.flags = ok ? sensor_->flags() : 0;
    s.alarms = rm_.alarms();
    s.volumeUl = rm_.volumeUl();
    s.volumeEstimated = rm_.volumeEstimated();
    s.runVolumeUl = rm_.runVolumeUl();
    s.plannedVolumeUl = rm_.plannedVolumeUl();
    s.targetVolumeUl = rm_.mode() == RunMode::VOLUME ? rm_.request().volumeUl : NAN;
    s.etaS = rm_.etaS();
    s.runElapsedS = rm_.runElapsedS();
    s.sensorStatus = sensor_ ? sensor_->status() : SensorStatus::ABSENT;
    if (sensor_) s.errors = sensor_->errors();
    s.zeroOffset = zeroOffset_;
    s.zeroing = rm_.zeroing();
    s.zeroProgress = rm_.zeroProgress(now);
    s.keepRunning = rm_.keepRunning();
    s.programStep = rm_.programStep();
    s.hasProfile = hasProfile_;
    s.profileMismatch = hasProfile_ && !profileMatchesDriver();
    strlcpy(s.profileId, profile_.id.c_str(), sizeof(s.profileId));
    strlcpy(s.profileName, profile_.name.c_str(), sizeof(s.profileName));
    s.hasCalibration = calUsable_;
    const bool fb = rm_.usesFeedback();
    s.ff = fb ? rm_.flowController().feedForward() : NAN;
    s.corr = fb ? rm_.flowController().correction() : NAN;
    const CalibrationSweep& sw = rm_.sweep();
    s.calPhase = sw.phase();
    s.calIndex = sw.currentIndex();
    s.calPoints = sw.pointCount();
    s.calPhaseElapsed = sw.running() ? sw.phaseElapsedS(now) : 0;
    s.calPhaseDuration = sw.phaseDurationS();
    s.filterWindowS = profile_.pid.filterWindowS;
    s.pressurePresent = psensor_ != nullptr;
    s.pressureAlarmKpa = pAlarm_;
    s.pressureZeroKpa = pZero_;
    if (psensor_) {
        s.pressureStatus = psensor_->status();
        s.pressureErrors = psensor_->errors();
        const bool pOk = s.pressureStatus == SensorStatus::OK && pAvg_.count() > 0;
        s.pressureKpa = pOk ? gaugeKpa() : NAN;
        s.pressureAbsKpa = pOk ? psensor_->absKpa() : NAN;
        s.pressureSaturated = psensor_->saturated();
        s.pressureStatusByte = psensor_->statusByte();
        s.pressureTaring = pTaring_;
        s.pressureTareProgress = pTaring_ && pTareLen_ > 0 ? std::fmin(1.0f, (now - pTareStart_) / pTareLen_) : 0;
    }
}
