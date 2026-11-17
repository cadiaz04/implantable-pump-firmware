#include "run_manager.h"
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

const char* channelStateName(ChannelState s) {
    switch (s) {
        case ChannelState::IDLE: return "IDLE";
        case ChannelState::RUNNING: return "RUNNING";
        case ChannelState::PAUSED: return "PAUSED";
        case ChannelState::CALIBRATING: return "CALIBRATING";
        case ChannelState::FAULT: return "FAULT";
        default: return "ESTOP";
    }
}

const char* runModeName(RunMode m) {
    switch (m) {
        case RunMode::MANUAL: return "manual";
        case RunMode::CLOSED_LOOP: return "closed_loop";
        case RunMode::OPEN_LOOP: return "open_loop";
        case RunMode::VOLUME: return "volume";
        case RunMode::PROGRAM: return "program";
        case RunMode::CALIBRATION: return "calibration";
        default: return "none";
    }
}

bool runModeFromName(const char* s, RunMode& out) {
    if (!s) return false;
    static const RunMode all[] = {RunMode::MANUAL, RunMode::CLOSED_LOOP, RunMode::OPEN_LOOP, RunMode::VOLUME, RunMode::PROGRAM};
    for (RunMode m : all) {
        if (!strcmp(s, runModeName(m))) {
            out = m;
            return true;
        }
    }
    return false;
}

void RunManager::configure(const RunConfig& cfg) {
    cfg_ = cfg;
    cfg_.flow.minCommand = cfg.minCommand;
    cfg_.flow.maxCommand = cfg.maxCommand;
    // Keep the active setpoint across a live re-configure (e.g. PID retune).
    const float sp = fc_.setpoint();
    const bool safe = fc_.isSafe();
    fc_.configure(cfg_.flow);
    if (safe) fc_.setSetpoint(sp);
    theoryModel_.setGain(cfg.theoreticalGain);
}

void RunManager::emit(uint8_t level, const char* fmt, ...) {
    RunEvent& e = events_[(evHead_ + evCount_) % kEvents];
    if (evCount_ == kEvents) evHead_ = (evHead_ + 1) % kEvents;
    else ++evCount_;
    e.level = level;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e.msg, sizeof(e.msg), fmt, ap);
    va_end(ap);
}

bool RunManager::popEvent(RunEvent& e) {
    if (evCount_ == 0) return false;
    e = events_[evHead_];
    evHead_ = (evHead_ + 1) % kEvents;
    --evCount_;
    return true;
}

const char* RunManager::validateFlow(float f) const {
    if (!std::isfinite(f)) return "flow setpoint is not a number";
    if (f < MIN_FLOW_UL_MIN || f > MAX_FLOW_UL_MIN) return "flow setpoint outside firmware limits (MIN/MAX_FLOW_RATE_ML_HR)";
    return nullptr;
}

const FlowModel* RunManager::openLoopModel() const {
    if (calModel_) return calModel_;
    if (cfg_.theoreticalGain > 0) return &theoryModel_;
    return nullptr;
}

float RunManager::estimateFlow(float command) const {
    if (calModel_) {
        const float f = calModel_->flowAt(command);
        return f > 0 ? f : 0;
    }
    return cfg_.theoreticalGain * command;
}

bool RunManager::usesFeedback() const {
    if (state_ != ChannelState::RUNNING && state_ != ChannelState::PAUSED) return false;
    switch (mode_) {
        case RunMode::CLOSED_LOOP: return true;
        case RunMode::VOLUME: return !req_.openLoop;
        case RunMode::PROGRAM:
            return stepIdx_ >= 0 && stepIdx_ < req_.program.n && req_.program.steps[stepIdx_].kind == StepKind::FLOW;
        default: return false;
    }
}

float RunManager::flowSetpoint() const {
    switch (mode_) {
        case RunMode::CLOSED_LOOP:
        case RunMode::OPEN_LOOP: return req_.flowUlMin;
        case RunMode::VOLUME: return req_.durationS > 0 ? req_.volumeUl / req_.durationS * 60.0f : NAN;
        case RunMode::PROGRAM:
            if (stepIdx_ >= 0 && stepIdx_ < req_.program.n && req_.program.steps[stepIdx_].kind != StepKind::MANUAL)
                return req_.program.steps[stepIdx_].value;
            return NAN;
        default: return NAN;
    }
}

float RunManager::plannedVolumeUl() const {
    if (mode_ != RunMode::VOLUME) return NAN;
    const float p = flowSetpoint() * runElapsedS_ / 60.0f;
    return p > req_.volumeUl ? req_.volumeUl : p;
}

float RunManager::etaS() const {
    if (state_ != ChannelState::RUNNING && state_ != ChannelState::PAUSED) return NAN;
    if (mode_ == RunMode::VOLUME) {
        const float f = flowSetpoint();
        if (!(f > 0)) return NAN;
        const double rem = req_.volumeUl - runTotal_.volumeUl();
        return rem > 0 ? static_cast<float>(rem / f * 60.0) : 0.0f;
    }
    if (mode_ == RunMode::PROGRAM && !req_.program.loop && stepIdx_ >= 0) {
        float rem = req_.program.steps[stepIdx_].durationS - stepElapsedS_;
        for (int i = stepIdx_ + 1; i < req_.program.n; ++i) rem += req_.program.steps[i].durationS;
        return rem > 0 ? rem : 0;
    }
    return NAN;
}

const char* RunManager::start(const RunRequest& r, float nowS, bool sensorPresent, bool sensorOk) {
    if (state_ == ChannelState::ESTOP) return "E-STOP is latched; reset it first";
    if (state_ == ChannelState::FAULT) return "channel is in FAULT; press Stop to acknowledge";
    if (state_ == ChannelState::CALIBRATING) return "calibration sweep running";
    if (zeroing_) return "zero/tare in progress";
    if (overpressure_) return "line pressure is above the alarm limit (occlusion?)";
    if (!(cfg_.maxCommand > 0)) return "pump profile has no max command";

    auto checkCmd = [&](float c) -> const char* {
        if (!std::isfinite(c) || c < 0 || c > cfg_.maxCommand) return "command outside 0..maxCommand of the pump profile";
        if (c > 0 && c < cfg_.minCommand) return "command below the pump's minimum command";
        return nullptr;
    };
    // Model used for feed-forward in feedback modes.
    auto closedModel = [&](float refFlow, float initialCmd) -> const FlowModel* {
        if (calModel_) return calModel_;
        if (initialCmd > 0 && refFlow > 0) {
            fallbackModel_.setGain(refFlow / initialCmd);
            return &fallbackModel_;
        }
        if (cfg_.theoreticalGain > 0) return &theoryModel_;
        return nullptr;
    };
    static const char* kNoModelClosed = "no calibration: enter an initial command (or calibrate the pump)";
    static const char* kNoModelOpen = "open-loop needs an active calibration (or syringe geometry)";

    bool needsSensor = false;
    const FlowModel* model = nullptr;
    const char* err = nullptr;
    if (r.initialCommand != 0 && (err = checkCmd(r.initialCommand))) return err;

    switch (r.mode) {
        case RunMode::MANUAL:
            if ((err = checkCmd(r.command))) return err;
            break;
        case RunMode::CLOSED_LOOP:
            if ((err = validateFlow(r.flowUlMin))) return err;
            needsSensor = true;
            if (!(model = closedModel(r.flowUlMin, r.initialCommand))) return kNoModelClosed;
            break;
        case RunMode::OPEN_LOOP:
            if ((err = validateFlow(r.flowUlMin))) return err;
            if (!(model = openLoopModel())) return kNoModelOpen;
            break;
        case RunMode::VOLUME: {
            if (!std::isfinite(r.volumeUl) || !(r.volumeUl > 0)) return "target volume must be > 0";
            if (!std::isfinite(r.durationS) || !(r.durationS > 0)) return "duration must be > 0";
            const float f = r.volumeUl / r.durationS * 60.0f;
            if ((err = validateFlow(f))) return err;
            if (r.openLoop) {
                if (!(model = openLoopModel())) return kNoModelOpen;
            } else {
                needsSensor = true;
                if (!(model = closedModel(f, r.initialCommand))) return kNoModelClosed;
            }
            break;
        }
        case RunMode::PROGRAM: {
            if (r.program.n < 1 || r.program.n > MAX_PROGRAM_STEPS) return "program needs 1-16 steps";
            for (int i = 0; i < r.program.n; ++i) {
                const ProgramStep& s = r.program.steps[i];
                if (!std::isfinite(s.durationS) || !(s.durationS > 0)) return "every program step needs a duration > 0";
                if (s.kind == StepKind::MANUAL) {
                    if ((err = checkCmd(s.value))) return err;
                } else {
                    if ((err = validateFlow(s.value))) return err;
                    if (s.kind == StepKind::FLOW) {
                        needsSensor = true;
                        const FlowModel* m = closedModel(s.value, r.initialCommand);
                        if (!m) return kNoModelClosed;
                        model = m;
                    } else if (!openLoopModel()) {
                        return kNoModelOpen;
                    }
                }
            }
            break;
        }
        default: return "invalid mode";
    }
    if (needsSensor && !sensorPresent) return "no flow sensor configured on this channel";
    if (needsSensor && !sensorOk) return "flow sensor is not OK";

    const bool live = (state_ == ChannelState::RUNNING || state_ == ChannelState::PAUSED) && mode_ == r.mode;
    if (!live) {
        runTotal_.reset();
        runElapsedS_ = 0;
        fc_.reset();
    }
    req_ = r;
    mode_ = r.mode;
    latched_ = 0;
    state_ = ChannelState::RUNNING;
    if (model) fc_.setModel(model);

    switch (mode_) {
        case RunMode::CLOSED_LOOP: fc_.setSetpoint(r.flowUlMin); break;
        case RunMode::VOLUME:
            if (!r.openLoop) fc_.setSetpoint(r.volumeUl / r.durationS * 60.0f);
            break;
        case RunMode::PROGRAM: enterStep(0, nowS); break;
        default: break;
    }
    if (live) emit(0, "Setpoint updated (%s)", runModeName(mode_));
    else emit(0, "Run started: %s%s", runModeName(mode_), r.keepRunning ? " (keep running if disconnected)" : "");
    return nullptr;
}

const char* RunManager::startCalibration(const SweepConfig& sweep, bool keepRunning, float nowS) {
    if (state_ == ChannelState::ESTOP) return "E-STOP is latched; reset it first";
    if (state_ != ChannelState::IDLE) return "channel must be IDLE to calibrate";
    if (zeroing_) return "zero/tare in progress";
    if (overpressure_) return "line pressure is above the alarm limit (occlusion?)";
    for (int i = 0; i < sweep.nCommands; ++i) {
        const float c = sweep.commands[i];
        if (!std::isfinite(c) || c < 0 || c > cfg_.maxCommand) return "sweep command outside 0..maxCommand";
    }
    const char* err = nullptr;
    if (!sweep_.start(sweep, nowS, &err)) return err;
    req_ = RunRequest{};
    req_.mode = RunMode::CALIBRATION;
    req_.keepRunning = keepRunning;
    req_.reverse = sweep.reverse;
    mode_ = RunMode::CALIBRATION;
    latched_ = 0;
    state_ = ChannelState::CALIBRATING;
    emit(0, "Calibration sweep started (%d points)", sweep_.pointCount());
    return nullptr;
}

const char* RunManager::retryCalibrationPoint(int index, bool keepRunning, float nowS) {
    if (state_ == ChannelState::ESTOP) return "E-STOP is latched; reset it first";
    if (state_ != ChannelState::IDLE) return "channel must be IDLE to retry a point";
    if (zeroing_) return "zero/tare in progress";
    if (!sweep_.retryPoint(index, nowS)) return "no such point (or no finished sweep)";
    req_ = RunRequest{};
    req_.mode = RunMode::CALIBRATION;
    req_.keepRunning = keepRunning;
    req_.reverse = sweep_.reverse();
    mode_ = RunMode::CALIBRATION;
    latched_ = 0;
    state_ = ChannelState::CALIBRATING;
    emit(0, "Re-measuring calibration point %d", index + 1);
    return nullptr;
}

const char* RunManager::pause(float nowS) {
    if (state_ != ChannelState::RUNNING) return "not running";
    state_ = ChannelState::PAUSED;
    emit(0, "Paused");
    return nullptr;
}

const char* RunManager::resume(float nowS, bool sensorOk, bool air) {
    if (state_ != ChannelState::PAUSED) return "not paused";
    if (usesFeedback() && (!sensorOk || air)) return "flow sensor fault / air-in-line still present";
    if (overpressure_) return "line pressure is still above the alarm limit (or the pressure sensor is faulted)";
    state_ = ChannelState::RUNNING;
    latched_ &= ~ALARM_NO_SENSOR;
    emit(0, "Resumed");
    return nullptr;
}

void RunManager::endRun(const char* msg, uint8_t level) {
    if (state_ == ChannelState::CALIBRATING) sweep_.abort(0);
    state_ = ChannelState::IDLE;
    mode_ = RunMode::NONE;
    stepIdx_ = -1;
    out_ = ChannelOutputs{};
    if (msg) emit(level, "%s", msg);
}

void RunManager::stop(float nowS, const char* reason) {
    if (state_ == ChannelState::ESTOP) return;
    if (state_ == ChannelState::IDLE) {
        latched_ = 0;   // second Stop acknowledges/clears latched alarms
        if (zeroing_) {
            zeroing_ = false;
            emit(0, "Zero/tare cancelled");
        }
        return;
    }
    if (state_ == ChannelState::CALIBRATING) sweep_.abort(nowS);
    endRun(reason ? reason : "Stopped", reason ? 1 : 0);
}

void RunManager::estop() {
    if (state_ == ChannelState::CALIBRATING) sweep_.abort(0);
    zeroing_ = false;
    state_ = ChannelState::ESTOP;
    mode_ = RunMode::NONE;
    stepIdx_ = -1;
    out_ = ChannelOutputs{};
    latched_ |= ALARM_ESTOP;
}

void RunManager::resetEstop() {
    if (state_ != ChannelState::ESTOP) return;
    state_ = ChannelState::IDLE;
    latched_ &= ~ALARM_ESTOP;
}

const char* RunManager::startZero(float seconds, float nowS) {
    if (state_ != ChannelState::IDLE) return "channel must be IDLE (pump stopped) to zero";
    if (zeroing_) return "zero already in progress";
    if (!(seconds >= 1 && seconds <= 600)) return "zero duration must be 1-600 s";
    zeroing_ = true;
    zeroStart_ = nowS;
    zeroLen_ = seconds;
    zeroSum_ = 0;
    zeroN_ = 0;
    zeroAir_ = false;
    emit(0, "Zero/tare started (%.0f s)", seconds);
    return nullptr;
}

float RunManager::zeroProgress(float nowS) const {
    if (!zeroing_ || zeroLen_ <= 0) return 0;
    const float p = (nowS - zeroStart_) / zeroLen_;
    return p > 1 ? 1 : p;
}

bool RunManager::takeZeroResult(float& offset, bool& ok) {
    if (!zeroReady_) return false;
    zeroReady_ = false;
    offset = zeroOffset_;
    ok = zeroOk_;
    return true;
}

void RunManager::resetVolume() {
    total_.reset();
    volumeEstimated_ = false;
}

void RunManager::enterStep(int i, float nowS) {
    stepIdx_ = i;
    stepElapsedS_ = 0;
    const ProgramStep& s = req_.program.steps[i];
    if (s.kind == StepKind::FLOW) {
        fc_.setSetpoint(s.value);
    } else if (s.kind == StepKind::FLOW_OPEN_LOOP) {
        const FlowModel* m = openLoopModel();
        stepCommand_ = (m && s.value > 0) ? m->commandFor(s.value, cfg_.minCommand, cfg_.maxCommand) : 0;
    } else {
        stepCommand_ = s.value;
    }
}

float RunManager::runMode(const ChannelInputs& in) {
    switch (mode_) {
        case RunMode::MANUAL: return req_.command;
        case RunMode::CLOSED_LOOP:
            if (!fc_.isSafe()) {
                endRun("Unsafe flow setpoint, stopped", 2);
                return 0;
            }
            return fc_.update(in.dtS, in.flowCtl, in.sensorOk);
        case RunMode::OPEN_LOOP: {
            const FlowModel* m = openLoopModel();
            if (!m || !(req_.flowUlMin > 0)) return 0;
            return m->commandFor(req_.flowUlMin, cfg_.minCommand, cfg_.maxCommand);
        }
        case RunMode::VOLUME: {
            if (runTotal_.volumeUl() >= req_.volumeUl) {
                char msg[80];
                snprintf(msg, sizeof(msg), "Target volume delivered (%.1f uL in %.0f min)", runTotal_.volumeUl(), runElapsedS_ / 60.0f);
                endRun(msg, 0);
                return 0;
            }
            if (req_.openLoop) {
                const FlowModel* m = openLoopModel();
                return m ? m->commandFor(flowSetpoint(), cfg_.minCommand, cfg_.maxCommand) : 0;
            }
            return fc_.update(in.dtS, in.flowCtl, in.sensorOk);
        }
        case RunMode::PROGRAM: {
            stepElapsedS_ += in.dtS;
            if (stepElapsedS_ >= req_.program.steps[stepIdx_].durationS) {
                int next = stepIdx_ + 1;
                if (next >= req_.program.n) {
                    if (!req_.program.loop) {
                        endRun("Program complete", 0);
                        return 0;
                    }
                    next = 0;
                }
                enterStep(next, in.nowS);
            }
            const ProgramStep& s = req_.program.steps[stepIdx_];
            if (s.kind == StepKind::FLOW) return fc_.update(in.dtS, in.flowCtl, in.sensorOk);
            return stepCommand_;
        }
        default: return 0;
    }
}

ChannelOutputs RunManager::tick(const ChannelInputs& in) {
    const float dt = in.dtS;
    ChannelOutputs o;

    // ---- live alarm conditions ----
    const bool sensorFault = in.sensorPresent && !in.sensorOk;
    if (in.sensorOk && in.air) airS_ += dt;
    else if (!in.air) airS_ = 0;
    const bool air = airS_ >= cfg_.airDebounceS;
    alarms_ = latched_;
    if (sensorFault) alarms_ |= ALARM_SENSOR_FAULT;
    if (air) alarms_ |= ALARM_AIR;
    if (in.sensorOk && in.highFlow) alarms_ |= ALARM_HIGH_FLOW;

    // ---- zero / tare ----
    if (zeroing_) {
        if (in.newSample && in.sensorOk) {
            zeroSum_ += in.flowRaw;
            ++zeroN_;
            if (in.air) zeroAir_ = true;
        }
        if (in.nowS - zeroStart_ >= zeroLen_) {
            zeroing_ = false;
            zeroReady_ = true;
            zeroOk_ = zeroN_ > 0 && !zeroAir_;
            zeroOffset_ = zeroN_ > 0 ? static_cast<float>(zeroSum_ / zeroN_) : 0.0f;
            if (zeroOk_) emit(0, "Zero complete: offset %.2f uL/min (%u samples)", zeroOffset_, static_cast<unsigned>(zeroN_));
            else emit(1, "Zero failed (%s); previous offset kept", zeroN_ == 0 ? "no sensor data" : "air-in-line");
        }
    }

    // ---- watchdog ----
    const bool active = state_ == ChannelState::RUNNING || state_ == ChannelState::PAUSED || state_ == ChannelState::CALIBRATING;
    if (active && in.heartbeatExpired && !req_.keepRunning) {
        latched_ |= ALARM_WATCHDOG;
        alarms_ |= ALARM_WATCHDOG;
        endRun("Browser heartbeat lost: run stopped by watchdog", 2);
    }

    // ---- firmware flow ceiling (all motor-active states) ----
    const bool motorState = state_ == ChannelState::RUNNING || state_ == ChannelState::CALIBRATING;
    if (motorState && in.sensorOk && in.flowAvg > MAX_FLOW_UL_MIN * FLOW_CEILING_TRIP_FACTOR) ceilingS_ += dt;
    else ceilingS_ = 0;
    if (ceilingS_ >= FLOW_CEILING_TRIP_S) {
        ceilingS_ = 0;
        if (state_ == ChannelState::CALIBRATING) sweep_.abort(in.nowS);
        state_ = ChannelState::FAULT;
        mode_ = RunMode::NONE;
        latched_ |= ALARM_FLOW_CEILING;
        alarms_ |= ALARM_FLOW_CEILING;
        emit(2, "FAULT: measured flow above firmware ceiling (%.0f uL/min)", MAX_FLOW_UL_MIN);
    }

    // ---- pressure / occlusion protection (all motor-active states) ----
    if (in.pressurePresent && !in.pressureOk) alarms_ |= ALARM_PRESSURE_SENSOR;
    bool high = false;
    if (cfg_.pressureAlarmKpa > 0 && in.pressurePresent) {
        // Fail closed: a saturated or faulted sensor can't prove the line is clear.
        high = !in.pressureOk || in.pressureSaturated || !(in.pressureKpa <= cfg_.pressureAlarmKpa);
        if (in.pressureOk && high) alarms_ |= ALARM_OVERPRESSURE;
    }
    if (high) overS_ += dt;
    else overS_ = 0;
    overpressure_ = overS_ >= cfg_.pressureDebounceS;
    // Re-check: the ceiling trip above may already have moved us to FAULT.
    const bool stillActive = state_ == ChannelState::RUNNING || state_ == ChannelState::CALIBRATING;
    if (overpressure_ && stillActive) {
        latched_ |= ALARM_OVERPRESSURE;
        alarms_ |= ALARM_OVERPRESSURE;
        char why[64];
        if (!in.pressureOk) snprintf(why, sizeof(why), "pressure sensor fault");
        else if (in.pressureSaturated) snprintf(why, sizeof(why), "pressure above sensor range");
        else snprintf(why, sizeof(why), "pressure %.1f kPa > limit %.1f kPa", in.pressureKpa, cfg_.pressureAlarmKpa);
        if (state_ == ChannelState::CALIBRATING) {
            sweep_.abort(in.nowS);
            state_ = ChannelState::IDLE;
            mode_ = RunMode::NONE;
            emit(2, "Calibration aborted: %s (occlusion?)", why);
        } else if (cfg_.sensorFaultStops) {
            char msg[96];   // events keep the first 79 chars
            snprintf(msg, sizeof(msg), "Stopped: %s (occlusion?)", why);
            endRun(msg, 2);
        } else {
            state_ = ChannelState::PAUSED;
            emit(2, "Paused: %s (occlusion?)", why);
        }
    }

    // ---- state behaviour ----
    switch (state_) {
        case ChannelState::RUNNING: {
            if (usesFeedback() && (sensorFault || air || !in.sensorPresent)) {
                if (!in.sensorPresent) latched_ |= ALARM_NO_SENSOR;
                const char* why = air ? "air-in-line" : "flow sensor fault";
                if (cfg_.sensorFaultStops) {
                    char msg[80];
                    snprintf(msg, sizeof(msg), "Stopped: %s", why);
                    endRun(msg, 2);
                } else {
                    state_ = ChannelState::PAUSED;
                    emit(2, "Paused: %s (press Resume when cleared)", why);
                }
                break;
            }
            runElapsedS_ += dt;
            const float c = runMode(in);
            if (state_ == ChannelState::RUNNING) {
                o.command = c;
                o.reverse = req_.reverse;
                o.active = true;
            }
            break;
        }
        case ChannelState::CALIBRATING: {
            if (sensorFault) {
                sweep_.abort(in.nowS);
                state_ = ChannelState::IDLE;
                mode_ = RunMode::NONE;
                emit(2, "Calibration aborted: flow sensor fault");
                break;
            }
            SweepSample s;
            s.valid = in.newSample && in.sensorOk;
            s.flowRaw = in.flowRaw;
            s.flow = in.flow;
            s.tempC = in.tempC;
            s.air = in.air;
            const float c = sweep_.tick(in.nowS, s);
            if (!sweep_.running()) {
                state_ = ChannelState::IDLE;
                mode_ = RunMode::NONE;
                emit(0, "Calibration sweep finished: %d/%d valid points", sweep_.validPointCount(), sweep_.pointCount());
            } else {
                o.command = c;
                o.reverse = sweep_.reverse();
                o.active = true;
            }
            break;
        }
        default: break;
    }
    if (!(o.command >= 0)) o.command = 0;   // also catches NaN
    if (o.command > cfg_.maxCommand) o.command = cfg_.maxCommand;
    out_ = o;

    // ---- totalizers ----
    if (o.command > 0 || in.actualCommand > 0) sinceActiveS_ = 0;
    else sinceActiveS_ += dt;
    const bool integrate = sinceActiveS_ <= cfg_.volumeTailS;
    const bool runIntegrate = mode_ == RunMode::VOLUME &&
                              (state_ == ChannelState::RUNNING || state_ == ChannelState::PAUSED);
    if (integrate) {
        if (in.sensorPresent && in.sensorOk) {
            if (in.newSample) {
                total_.add(in.nowS, in.flow);
                if (runIntegrate) runTotal_.add(in.nowS, in.flow);
            }
        } else {
            const float est = estimateFlow(in.actualCommand);
            total_.add(in.nowS, est);
            if (runIntegrate) runTotal_.add(in.nowS, est);
            volumeEstimated_ = true;
        }
    } else {
        total_.breakChain();
        runTotal_.breakChain();
    }
    return o;
}
