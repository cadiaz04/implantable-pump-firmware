#include "profile_json.h"
#include <cmath>
#include "config.h"

namespace {

void num(JsonObject o, const char* k, float v) {
    if (std::isfinite(v)) o[k] = v;
    else o[k] = nullptr;
}

float getf(JsonObjectConst o, const char* k, float def) {
    JsonVariantConst v = o[k];
    if (v.isNull()) return def;
    if (v.is<float>()) return v.as<float>();
    return NAN;
}

bool inRange(float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; }

bool req(bool cond, String& err, const char* msg) {
    if (!cond) err = msg;
    return cond;
}

}  // namespace

void fitToJson(const CurveFit& f, JsonObject o) {
    o["fitType"] = fitTypeId(f.type);
    JsonArray c = o["coefficients"].to<JsonArray>();
    for (float v : f.c) c.add(v);
    if (f.type == FitType::LOOKUP) {
        JsonObject lk = o["lookup"].to<JsonObject>();
        JsonArray x = lk["x"].to<JsonArray>();
        JsonArray y = lk["y"].to<JsonArray>();
        for (int i = 0; i < f.n; ++i) {
            x.add(f.lx[i]);
            y.add(f.ly[i]);
        }
    }
    num(o, "xMin", f.xMin);
    num(o, "xMax", f.xMax);
    num(o, "r2", f.r2);
    num(o, "rmse", f.rmse);
    num(o, "maxErrPct", f.maxErrPct);
    o["nonMonotone"] = f.nonMonotone;
}

void calibrationToJson(const Calibration& c, JsonObject o) {
    o["id"] = c.id;
    o["name"] = c.name;
    o["timestamp"] = c.timestamp;
    o["fluid"] = c.fluid;
    o["tubingNote"] = c.tubingNote;
    fitToJson(c.fit, o);
    o["active"] = c.active;
    JsonArray pts = o["points"].to<JsonArray>();
    for (const auto& p : c.points) {
        JsonObject j = pts.add<JsonObject>();
        num(j, "command", p.command);
        num(j, "flow", p.flow);
        num(j, "ref", p.ref);
        num(j, "std", p.std);
        num(j, "temp", p.tempC);
        j["n"] = p.n;
        j["pass"] = p.pass;
        j["status"] = p.status;
    }
}

bool calibrationFromJson(JsonObjectConst o, Calibration& c, String& err) {
    c = Calibration{};
    c.id = o["id"] | "";
    c.name = o["name"] | "";
    c.timestamp = o["timestamp"] | 0u;
    c.fluid = o["fluid"] | "water";
    c.tubingNote = o["tubingNote"] | "";
    c.active = o["active"] | false;
    if (c.id.empty()) c.id = "c" + std::to_string(c.timestamp);
    FitType ft;
    if (!fitTypeFromId(o["fitType"] | "linear", ft)) {
        err = "calibration: unknown fitType";
        return false;
    }
    CurveFit& f = c.fit;
    f.type = ft;
    JsonArrayConst co = o["coefficients"];
    for (int i = 0; i < 3 && i < static_cast<int>(co.size()); ++i) f.c[i] = co[i] | 0.0f;
    if (ft == FitType::LOOKUP) {
        JsonArrayConst x = o["lookup"]["x"];
        JsonArrayConst y = o["lookup"]["y"];
        f.n = 0;
        for (size_t i = 0; i < x.size() && i < y.size() && f.n < MAX_LOOKUP_POINTS; ++i) {
            f.lx[f.n] = x[i] | 0.0f;
            f.ly[f.n] = y[i] | 0.0f;
            ++f.n;
        }
        if (f.n < 2) {
            err = "calibration: lookup needs >= 2 points";
            return false;
        }
    }
    f.xMin = getf(o, "xMin", 0);
    f.xMax = getf(o, "xMax", 0);
    f.r2 = getf(o, "r2", NAN);
    f.rmse = getf(o, "rmse", NAN);
    f.maxErrPct = getf(o, "maxErrPct", NAN);
    f.nonMonotone = o["nonMonotone"] | false;
    f.ok = true;
    for (JsonObjectConst j : o["points"].as<JsonArrayConst>()) {
        CalPointRec p;
        p.command = getf(j, "command", NAN);
        p.flow = getf(j, "flow", NAN);
        p.ref = getf(j, "ref", NAN);
        p.std = getf(j, "std", NAN);
        p.tempC = getf(j, "temp", NAN);
        p.n = j["n"] | 0u;
        p.pass = j["pass"] | 0;
        p.status = j["status"] | "ok";
        c.points.push_back(p);
        if (c.points.size() >= 64) break;
    }
    return true;
}

void profileToJson(const PumpProfile& p, JsonObject o) {
    o["id"] = p.id;
    o["name"] = p.name;
    o["pumpType"] = p.pumpType;
    o["notes"] = p.notes;
    JsonObject m = o["motor"].to<JsonObject>();
    m["driverType"] = driverTypeId(p.motor.driverType);
    m["stepsPerRev"] = p.motor.stepsPerRev;
    m["microstep"] = p.motor.microstep;
    m["gearRatio"] = p.motor.gearRatio;
    m["maxCommand"] = p.motor.maxCommand;
    m["minCommand"] = p.motor.minCommand;
    m["accel"] = p.motor.accel;
    m["invertDir"] = p.motor.invertDir;
    m["decay"] = p.motor.decay == DecayMode::SLOW ? "slow" : "fast";
    m["minDuty"] = p.motor.minDuty;
    m["kickMs"] = p.motor.kickMs;
    m["kickDuty"] = p.motor.kickDuty;
    m["releaseWhenStopped"] = p.motor.releaseWhenStopped;
    m["commandUnit"] = commandUnitFor(p.motor.driverType);
    JsonObject s = o["syringe"].to<JsonObject>();
    s["innerDiameter_mm"] = p.syringeIdMm;
    s["leadscrewPitch_mm"] = p.leadscrewPitchMm;
    num(s, "theoretical_uLmin_per_rpm", p.theoreticalGain());
    o["fluid"] = p.fluid;
    JsonObject pid = o["pid"].to<JsonObject>();
    pid["kp"] = p.pid.kp;
    pid["ki"] = p.pid.ki;
    pid["kd"] = p.pid.kd;
    pid["loopHz"] = p.pid.loopHz;
    pid["filterWindowS"] = p.pid.filterWindowS;
    pid["filter"] = p.pid.filterEma ? "ema" : "ma";
    pid["outputLimitPct"] = p.pid.outputLimitPct;
    pid["integralLimitPct"] = p.pid.integralLimitPct;
    JsonArray cals = o["calibrations"].to<JsonArray>();
    for (const auto& c : p.calibrations) calibrationToJson(c, cals.add<JsonObject>());
}

bool profileFromJson(JsonObjectConst o, PumpProfile& p, String& err) {
    PumpProfile out;
    out.id = o["id"] | "";
    out.name = o["name"] | "";
    if (!req(!out.name.empty() && out.name.size() <= 63, err, "name is required (max 63 chars)")) return false;
    out.pumpType = o["pumpType"] | "other";
    if (out.pumpType != "syringe" && out.pumpType != "diaphragm" && out.pumpType != "peristaltic" && out.pumpType != "other") {
        err = "pumpType must be syringe, diaphragm, peristaltic or other";
        return false;
    }
    out.notes = o["notes"] | "";
    if (out.notes.size() > 500) out.notes.resize(500);

    JsonObjectConst m = o["motor"];
    if (!req(!m.isNull(), err, "motor settings missing")) return false;
    out.motor.driverType = driverTypeFromId(m["driverType"] | "");
    if (!req(out.motor.driverType != DriverType::NONE, err, "motor.driverType must be uln2003, stepdir, drv8871 or pwmdir")) return false;
    const bool dc = familyOf(out.motor.driverType) == DriverFamily::DC;
    MotorParams& mp = out.motor;
    mp.stepsPerRev = getf(m, "stepsPerRev", mp.stepsPerRev);
    mp.microstep = getf(m, "microstep", dc ? 1 : mp.microstep);
    mp.gearRatio = getf(m, "gearRatio", 1);
    mp.maxCommand = getf(m, "maxCommand", NAN);
    mp.minCommand = getf(m, "minCommand", 0);
    mp.accel = getf(m, "accel", mp.accel);
    mp.invertDir = m["invertDir"] | false;
    mp.decay = strcmp(m["decay"] | "fast", "slow") == 0 ? DecayMode::SLOW : DecayMode::FAST;
    mp.minDuty = getf(m, "minDuty", 0);
    mp.kickMs = static_cast<uint16_t>(m["kickMs"] | 0);
    mp.kickDuty = getf(m, "kickDuty", 80);
    mp.releaseWhenStopped = m["releaseWhenStopped"] | true;
    const float absMax = dc ? ABS_MAX_DUTY_PCT : ABS_MAX_STEPPER_RPM;
    if (!req(inRange(mp.stepsPerRev, 1, 1e5f), err, "stepsPerRev must be 1-100000")) return false;
    if (!req(inRange(mp.microstep, 1, 256), err, "microstep must be 1-256")) return false;
    if (!req(inRange(mp.gearRatio, 0.001f, 1000), err, "gearRatio must be 0.001-1000")) return false;
    if (!req(inRange(mp.maxCommand, 0.0001f, absMax), err, dc ? "maxCommand must be 0-100 %" : "maxCommand must be 0-600 RPM")) return false;
    if (!req(inRange(mp.minCommand, 0, mp.maxCommand), err, "minCommand must be between 0 and maxCommand")) return false;
    if (!req(inRange(mp.accel, 0, 1e4f), err, "accel must be 0-10000 (0 = no ramp)")) return false;
    if (!req(inRange(mp.minDuty, 0, 100), err, "minDuty must be 0-100 %")) return false;
    if (!req(mp.kickMs <= 2000, err, "kickMs must be 0-2000")) return false;
    if (!req(inRange(mp.kickDuty, 0, 100), err, "kickDuty must be 0-100 %")) return false;

    JsonObjectConst s = o["syringe"];
    out.syringeIdMm = s.isNull() ? 0 : getf(s, "innerDiameter_mm", 0);
    out.leadscrewPitchMm = s.isNull() ? 0 : getf(s, "leadscrewPitch_mm", 0);
    if (!req(inRange(out.syringeIdMm, 0, 100) && inRange(out.leadscrewPitchMm, 0, 100), err, "syringe geometry must be 0-100 mm")) return false;

    out.fluid = o["fluid"] | "water";
    if (!req(out.fluid == "water" || out.fluid == "saline", err, "fluid must be water or saline")) return false;

    JsonObjectConst pid = o["pid"];
    if (!pid.isNull()) {
        PidParams& q = out.pid;
        q.kp = getf(pid, "kp", q.kp);
        q.ki = getf(pid, "ki", q.ki);
        q.kd = getf(pid, "kd", q.kd);
        q.loopHz = getf(pid, "loopHz", q.loopHz);
        q.filterWindowS = getf(pid, "filterWindowS", q.filterWindowS);
        q.filterEma = strcmp(pid["filter"] | "ma", "ema") == 0;
        q.outputLimitPct = getf(pid, "outputLimitPct", q.outputLimitPct);
        q.integralLimitPct = getf(pid, "integralLimitPct", q.integralLimitPct);
        if (!req(inRange(q.kp, 0, 100) && inRange(q.ki, 0, 100) && inRange(q.kd, 0, 1000), err, "PID gains must be >= 0")) return false;
        if (!req(inRange(q.loopHz, 0.2f, 10), err, "loopHz must be 0.2-10")) return false;
        if (!req(inRange(q.filterWindowS, 0.5f, 900), err, "filterWindowS must be 0.5-900 s")) return false;
        if (!req(inRange(q.outputLimitPct, 1, 1000) && inRange(q.integralLimitPct, 1, 1000), err, "PID limits must be 1-1000 %")) return false;
    }

    bool haveActive = false;
    for (JsonObjectConst c : o["calibrations"].as<JsonArrayConst>()) {
        Calibration cal;
        if (!calibrationFromJson(c, cal, err)) return false;
        if (cal.active) {
            if (haveActive) cal.active = false;
            haveActive = true;
        }
        out.calibrations.push_back(cal);
        if (out.calibrations.size() >= 10) break;
    }
    p = out;
    return true;
}
