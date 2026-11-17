#include "api.h"
#include <LittleFS.h>
#include <WiFi.h>
#include <cmath>
#include <memory>
#include "app/app.h"
#include "config/pin_rules.h"
#include "net/wifi_manager.h"
#include "pumps/profile_json.h"
#include "pumps/profile_store.h"
#include "sensors/sensor_registry.h"

namespace api {
namespace {

constexpr size_t kMaxBody = 32 * 1024;

// The app mutex is held for < ~1 ms by the real-time task; failing to get it
// within the timeout means something is wrong, so refuse rather than race.
#define LOCK_OR_503(req)                              \
    app::Lock l;                                      \
    if (!l.ok) {                                      \
        sendError(req, 503, "busy, try again");       \
        return;                                       \
    }
#define LOCK_OR_BUSY()             \
    app::Lock l;                   \
    if (!l.ok) {                   \
        code = 503;                \
        return "busy, try again";  \
    }

// ---------------------------------------------------------------- helpers

void sendDoc(AsyncWebServerRequest* r, int code, const JsonDocument& d) {
    AsyncResponseStream* resp = r->beginResponseStream("application/json");
    resp->setCode(code);
    resp->addHeader("Cache-Control", "no-store");
    serializeJson(d, *resp);
    r->send(resp);
}

void sendError(AsyncWebServerRequest* r, int code, const String& msg) {
    JsonDocument d;
    d["ok"] = false;
    d["error"] = msg;
    sendDoc(r, code, d);
}

void sendOk(AsyncWebServerRequest* r) {
    JsonDocument d;
    d["ok"] = true;
    sendDoc(r, 200, d);
}

void onBody(AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t index, size_t total) {
    if (total > kMaxBody) return;
    if (index == 0) {
        free(r->_tempObject);
        r->_tempObject = malloc(total + 1);
    }
    if (!r->_tempObject) return;
    memcpy(static_cast<uint8_t*>(r->_tempObject) + index, data, len);
    if (index + len == total) static_cast<char*>(r->_tempObject)[total] = 0;
}

// Splits "/api/a/b/c" → {"a","b","c"}.
int splitPath(const String& url, String* seg, int max) {
    int n = 0;
    int start = 5;   // after "/api/"
    if (url.length() <= 5) return 0;
    while (start < static_cast<int>(url.length()) && n < max) {
        int slash = url.indexOf('/', start);
        if (slash < 0) slash = url.length();
        if (slash > start) seg[n++] = url.substring(start, slash);
        start = slash + 1;
    }
    return n;
}

int parseChannel(const String& s) {
    const int n = s.toInt();
    return (n >= 1 && n <= MAX_CHANNELS) ? n - 1 : -1;
}

const char* alarmName(int bit) {
    switch (1 << bit) {
        case ALARM_AIR: return "air_in_line";
        case ALARM_SENSOR_FAULT: return "sensor_fault";
        case ALARM_HIGH_FLOW: return "high_flow";
        case ALARM_FLOW_CEILING: return "flow_ceiling";
        case ALARM_WATCHDOG: return "watchdog";
        case ALARM_ESTOP: return "estop";
        case ALARM_NO_SENSOR: return "no_sensor";
        case ALARM_OVERPRESSURE: return "overpressure";
        case ALARM_PRESSURE_SENSOR: return "pressure_sensor_fault";
        default: return "unknown";
    }
}

void hwToJson(const ChannelHw& hw, JsonObject o) {
    o["driver"] = driverTypeId(hw.driver);
    o["commandUnit"] = commandUnitFor(hw.driver);
    JsonArray pins = o["pins"].to<JsonArray>();
    JsonArray labels = o["pinLabels"].to<JsonArray>();
    for (int s = 0; s < maxPinCount(hw.driver); ++s) {
        pins.add(hw.pins[s]);
        labels.add(pinLabel(hw.driver, s));
    }
    o["requiredPins"] = requiredPinCount(hw.driver);
    o["enActiveLow"] = hw.enActiveLow;
    o["sensor"] = hw.sensor == SensorType::SLF3S ? "slf3s" : "none";
    o["pressure"] = hw.pressure == PressureType::MPRLS ? "mprls" : "none";
    o["sda"] = hw.sda;
    o["scl"] = hw.scl;
}

void profileBrief(const PumpProfile& p, JsonObject o) {
    JsonDocument tmp;
    profileToJson(p, tmp.to<JsonObject>());
    tmp.remove("calibrations");
    o.set(tmp.as<JsonObjectConst>());
    o["calibrationCount"] = p.calibrations.size();
    if (const Calibration* c = p.activeCalibration()) {
        JsonObject a = o["activeCalibration"].to<JsonObject>();
        a["id"] = c->id;
        a["name"] = c->name;
        a["timestamp"] = c->timestamp;
        fitToJson(c->fit, a);
    } else {
        o["activeCalibration"] = nullptr;
    }
}

const char* const kPressureUnits[4] = {"kPa", "psi", "mmHg", "cmH2O"};

// Gauge pressure is always reported in kPa; the UI converts for display.
void pressureToJson(const ChannelSnapshot& s, JsonObject o) {
    o["pressurePresent"] = s.pressurePresent;
    if (!s.pressurePresent) return;
    o["pressure_kPa"] = s.pressureKpa;
    o["pressureAbs_kPa"] = s.pressureAbsKpa;
    o["pressureZero_kPa"] = s.pressureZeroKpa;   // null = never tared (standard atmosphere used)
    o["pressureTared"] = std::isfinite(s.pressureZeroKpa);
    o["pressureSensor"] = sensorStatusName(s.pressureStatus);
    o["pressureSaturated"] = s.pressureSaturated;
    o["pressureStatusByte"] = s.pressureStatusByte;
    o["pressureTaring"] = s.pressureTaring;
    o["pressureTareProgress"] = s.pressureTareProgress;
    o["pressureAlarm_kPa"] = s.pressureAlarmKpa;
    JsonObject e = o["pressureErrors"].to<JsonObject>();
    e["frame"] = s.pressureErrors.crc;
    e["i2c"] = s.pressureErrors.i2c;
    e["consecutive"] = s.pressureErrors.consecutive;
    e["reinits"] = s.pressureErrors.reinits;
}

void snapshotToJson(const ChannelSnapshot& s, JsonObject o) {
    o["state"] = channelStateName(s.state);
    o["mode"] = runModeName(s.mode);
    o["driver"] = driverTypeId(s.driver);
    o["commandUnit"] = commandUnitFor(s.driver);
    o["command"] = s.command;
    o["commandActual"] = s.commandActual;
    o["stepRate_Hz"] = s.stepRateHz;
    o["setpoint"] = s.setpoint;
    o["manualCommand"] = s.manualCommand;
    o["reverse"] = s.reverse;
    o["flow_uLmin"] = s.flow;
    o["flow_avg_uLmin"] = s.flowAvg;
    o["flow_std"] = s.flowStd;
    o["temp_C"] = s.tempC;
    o["flags"] = s.flags;
    o["air"] = (s.alarms & ALARM_AIR) != 0;
    o["volume_uL"] = s.volumeUl;
    o["volumeEstimated"] = s.volumeEstimated;
    o["runVolume_uL"] = s.runVolumeUl;
    o["plannedVolume_uL"] = s.plannedVolumeUl;
    o["targetVolume_uL"] = s.targetVolumeUl;
    o["eta_s"] = s.etaS;
    o["elapsed_s"] = s.runElapsedS;
    JsonArray al = o["alarms"].to<JsonArray>();
    for (int b = 0; b < 16; ++b)
        if (s.alarms & (1 << b)) al.add(alarmName(b));
    o["sensor"] = sensorStatusName(s.sensorStatus);
    JsonObject e = o["errors"].to<JsonObject>();
    e["crc"] = s.errors.crc;
    e["i2c"] = s.errors.i2c;
    e["consecutive"] = s.errors.consecutive;
    e["reinits"] = s.errors.reinits;
    o["zeroOffset"] = s.zeroOffset;
    o["zeroing"] = s.zeroing;
    o["zeroProgress"] = s.zeroProgress;
    o["keepRunning"] = s.keepRunning;
    o["programStep"] = s.programStep;
    o["profileId"] = s.profileId;
    o["profileName"] = s.profileName;
    o["profileMismatch"] = s.profileMismatch;
    o["hasCalibration"] = s.hasCalibration;
    o["ff"] = s.ff;
    o["corr"] = s.corr;
    o["filterWindowS"] = s.filterWindowS;
    pressureToJson(s, o);
    JsonObject cal = o["cal"].to<JsonObject>();
    cal["phase"] = sweepPhaseName(s.calPhase);
    cal["index"] = s.calIndex;
    cal["points"] = s.calPoints;
    cal["elapsed"] = s.calPhaseElapsed;
    cal["duration"] = s.calPhaseDuration;
}

void signalsToJson(JsonArray arr) {
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        const int n = i + 1;
        const char* unit = commandUnitFor(app::hardware.ch[i].driver);
        auto add = [&](const char* p, const char* name, const char* u) {
            JsonObject s = arr.add<JsonObject>();
            s["id"] = String(String(p) + n);
            s["name"] = String(String(name) + " Ch" + n);
            s["unit"] = u;
            s["channel"] = n;
        };
        add("f", "Flow", "uL/min");
        add("fa", "Flow avg", "uL/min");
        add("sp", "Setpoint", "uL/min");
        add("c", "Command", unit);
        add("t", "Temp", "C");
        add("v", "Volume", "uL");
        if (app::hardware.ch[i].pressure != PressureType::NONE) add("p", "Pressure", "kPa");
    }
    for (int k = 0; k < g_sensors.count(); ++k) {
        Sensor* s = g_sensors.at(k);
        for (int i = 0; i < s->signalCount(); ++i) {
            const SignalInfo si = s->signalInfo(i);
            JsonObject o = arr.add<JsonObject>();
            o["id"] = si.id;
            o["name"] = si.name;
            o["unit"] = si.unit;
            o["channel"] = si.channel >= 0 ? si.channel + 1 : 0;
        }
    }
}

void settingsToJson(JsonObject o) {
    const AppSettings& s = app::settings;
    o["watchdogS"] = s.watchdogS;
    o["telemetryHz"] = s.telemetryHz;
    o["flowUnit"] = s.flowUnit ? "mL/min" : "uL/min";
    o["pressureUnit"] = kPressureUnits[s.pressureUnit < 4 ? s.pressureUnit : 0];
    o["sensorFaultAction"] = s.sensorFaultStops ? "stop" : "pause";
    o["hostname"] = s.hostname;
    JsonObject w = o["wifi"].to<JsonObject>();
    w["ssid"] = s.staSsid;
    w["passwordSet"] = s.staPass[0] != 0;
    w["apSsid"] = wifi_manager::apSsid();
    w["apIp"] = wifi_manager::apIp();
    w["staConnected"] = wifi_manager::staConnected();
    w["staIp"] = wifi_manager::staIp();
    w["apClients"] = wifi_manager::apClients();
}

void buildState(JsonDocument& d) {
    const uint32_t now = millis();
    d["ok"] = true;
    d["sim"] = app::simMode();
    d["version"] = app::firmwareVersion();
    d["uptimeMs"] = now;
    JsonObject t = d["time"].to<JsonObject>();
    t["epochMs"] = app::epochMs();
    t["synced"] = app::timeSynced();
    d["maxChannels"] = MAX_CHANNELS;
    JsonObject lim = d["limits"].to<JsonObject>();
    lim["maxFlow_uLmin"] = MAX_FLOW_UL_MIN;
    lim["minFlow_uLmin"] = MIN_FLOW_UL_MIN;
    lim["ceilingTripFactor"] = FLOW_CEILING_TRIP_FACTOR;
    lim["sensorFullScale_uLmin"] = 40000;
    d["heapFree"] = ESP.getFreeHeap();
    JsonObject h = d["history"].to<JsonObject>();
    h["capacity"] = app::historyCapacity();
    h["size"] = app::historySize();
    h["periodMs"] = HISTORY_PERIOD_MS;

    app::Lock l;
    if (!l.ok) {
        d["ok"] = false;
        d["error"] = "busy, try again";
        return;
    }
    d["estop"] = app::estopLatched;
    d["hbAge_s"] = app::watchdog.ageS(now);
    settingsToJson(d["settings"].to<JsonObject>());
    JsonArray chs = d["channels"].to<JsonArray>();
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        Channel& c = app::channels[i];
        JsonObject o = chs.add<JsonObject>();
        o["n"] = i + 1;
        hwToJson(app::hardware.ch[i], o["hw"].to<JsonObject>());
        if (c.hasProfile()) profileBrief(c.profile(), o["profile"].to<JsonObject>());
        else o["profile"] = nullptr;
        ChannelSnapshot s;
        c.snapshot(s, now);
        snapshotToJson(s, o["status"].to<JsonObject>());
    }
    signalsToJson(d["signals"].to<JsonArray>());
}

// ------------------------------------------------------------ run parsing

float numOr(JsonVariantConst v, float def) {
    if (v.isNull()) return def;
    return v.is<float>() ? v.as<float>() : NAN;
}

bool parseRun(JsonVariantConst b, RunRequest& r, String& err) {
    RunMode m;
    if (!runModeFromName(b["mode"] | "manual", m)) {
        err = "mode must be manual, closed_loop, open_loop, volume or program";
        return false;
    }
    r.mode = m;
    r.reverse = b["reverse"] | false;
    r.keepRunning = b["keepRunning"] | false;
    r.command = numOr(b["command"], 0);
    r.flowUlMin = numOr(b["flow_uLmin"], NAN);
    if (std::isnan(r.flowUlMin) && !b["flow_mLmin"].isNull()) r.flowUlMin = numOr(b["flow_mLmin"], NAN) * 1000.0f;
    r.volumeUl = numOr(b["volume_uL"], NAN);
    if (std::isnan(r.volumeUl) && !b["volume_mL"].isNull()) r.volumeUl = numOr(b["volume_mL"], NAN) * 1000.0f;
    r.durationS = numOr(b["duration_s"], NAN);
    if (std::isnan(r.durationS) && !b["duration_h"].isNull()) r.durationS = numOr(b["duration_h"], NAN) * 3600.0f;
    r.openLoop = b["openLoop"] | false;
    r.initialCommand = numOr(b["initialCommand"], 0);
    if (m == RunMode::PROGRAM) {
        JsonVariantConst p = b["program"];
        JsonArrayConst steps = p["steps"];
        r.program.loop = p["loop"] | false;
        r.program.n = 0;
        for (JsonObjectConst s : steps) {
            if (r.program.n >= MAX_PROGRAM_STEPS) {
                err = "max 16 program steps";
                return false;
            }
            ProgramStep& st = r.program.steps[r.program.n++];
            const char* k = s["kind"] | "manual";
            if (!strcmp(k, "manual")) st.kind = StepKind::MANUAL;
            else if (!strcmp(k, "flow")) st.kind = StepKind::FLOW;
            else if (!strcmp(k, "flow_ol")) st.kind = StepKind::FLOW_OPEN_LOOP;
            else {
                err = "step kind must be manual, flow or flow_ol";
                return false;
            }
            st.value = numOr(s["value"], NAN);
            st.durationS = numOr(s["durationS"], NAN);
        }
    }
    return true;
}

bool parseSweep(JsonVariantConst b, SweepConfig& c, String& err, float maxCmd) {
    JsonArrayConst cmds = b["commands"];
    if (!cmds.isNull() && cmds.size() > 0) {
        c.nCommands = 0;
        for (JsonVariantConst v : cmds) {
            if (c.nCommands >= MAX_SWEEP_COMMANDS) break;
            c.commands[c.nCommands++] = v.as<float>();
        }
    } else {
        const float lo = numOr(b["min"], 0);
        const float hi = numOr(b["max"], maxCmd);
        const int n = b["points"] | 10;
        if (!(hi > lo) || n < 2) {
            err = "need min < max and >= 2 points";
            return false;
        }
        c.nCommands = linearCommands(lo, hi, n, c.commands);
    }
    c.settleS = numOr(b["settleS"], c.settleS);
    c.averageS = numOr(b["averageS"], c.averageS);
    c.repeats = b["repeats"] | 1;
    c.upDown = b["upDown"] | false;
    c.zeroFirst = b["zeroFirst"] | true;
    c.zeroSettleS = numOr(b["zeroSettleS"], c.zeroSettleS);
    c.zeroS = numOr(b["zeroS"], c.zeroS);
    c.reverse = b["reverse"] | false;
    c.steadyWindowS = numOr(b["steadyWindowS"], c.steadyWindowS);
    c.steadyTimeoutS = numOr(b["steadyTimeoutS"], c.steadyTimeoutS);
    c.maxSlopePctPerMin = numOr(b["maxSlopePctPerMin"], c.maxSlopePctPerMin);
    c.maxStdPct = numOr(b["maxStdPct"], c.maxStdPct);
    c.stdFloorUlMin = numOr(b["stdFloor_uLmin"], c.stdFloorUlMin);
    c.slopeFloorUlMinPerMin = numOr(b["slopeFloor_uLmin_per_min"], c.slopeFloorUlMinPerMin);
    if (!(c.settleS >= 0 && c.settleS <= 3600) || !(c.averageS >= 1 && c.averageS <= 3600)) {
        err = "settleS 0-3600, averageS 1-3600";
        return false;
    }
    if (!(c.zeroS >= 1 && c.zeroS <= 600) || !(c.zeroSettleS >= 0 && c.zeroSettleS <= 600)) {
        err = "zeroS 1-600, zeroSettleS 0-600";
        return false;
    }
    if (!(c.steadyTimeoutS >= 0 && c.steadyTimeoutS <= 7200) || !(c.steadyWindowS >= 2 && c.steadyWindowS <= 900)) {
        err = "steadyTimeoutS 0-7200, steadyWindowS 2-900";
        return false;
    }
    if (c.repeats < 1 || c.repeats > 5) {
        err = "repeats 1-5";
        return false;
    }
    return true;
}

}  // namespace

// --------------------------------------------------------- channel actions

const char* channelAction(int ch, const char* action, JsonVariantConst body, int& code) {
    static String err;   // AsyncTCP task only
    code = 400;
    if (ch < 0 || ch >= MAX_CHANNELS) {
        code = 404;
        return "no such channel";
    }
    const uint32_t now = millis();
    if (!strcmp(action, "run")) {
        RunRequest r;
        if (!parseRun(body, r, err)) return err.c_str();
        LOCK_OR_BUSY();
        if (app::estopLatched) {
            code = 409;
            return "E-STOP is latched; reset it first";
        }
        if (const char* e = app::channels[ch].run(r, now)) {
            code = 409;
            return e;
        }
        app::watchdog.feed(now);   // starting a run counts as a heartbeat
        code = 200;
        return nullptr;
    }
    if (!strcmp(action, "stop")) {
        LOCK_OR_BUSY();
        app::channels[ch].stop(now);
        code = 200;
        return nullptr;
    }
    if (!strcmp(action, "pause") || !strcmp(action, "resume")) {
        LOCK_OR_BUSY();
        const char* e = action[0] == 'p' ? app::channels[ch].pause(now) : app::channels[ch].resume(now);
        code = e ? 409 : 200;
        return e;
    }
    if (!strcmp(action, "volume-reset")) {
        LOCK_OR_BUSY();
        app::channels[ch].resetVolume();
        code = 200;
        return nullptr;
    }
    if (!strcmp(action, "profile")) {
        const String id = body["id"] | "";
        if (!ProfileStore::validId(id) || !g_profiles.exists(id)) {
            code = 404;
            return "unknown profile id";
        }
        DriverType pd = DriverType::NONE;
        for (const auto& s : g_profiles.list())
            if (s.id == id) pd = s.driver;
        LOCK_OR_BUSY();
        if (!app::channels[ch].isIdle()) {
            code = 409;
            return "channel must be idle to change its pump profile";
        }
        if (familyOf(pd) != familyOf(app::hardware.ch[ch].driver)) {
            code = 409;
            return "profile is for a different motor type (stepper vs DC) than this channel's driver";
        }
        app::Job* j = new app::Job(app::JobType::ASSIGN_PROFILE);
        j->ch = ch;
        j->id = id;
        if (!app::enqueue(j)) {
            code = 503;
            return "busy";
        }
        code = 200;
        return nullptr;
    }
    code = 404;
    return "unknown action";
}

namespace {

// ------------------------------------------------------------- handlers

void handlePumps(AsyncWebServerRequest* r, String* seg, int n, JsonDocument& body) {
    const auto method = r->method();
    if (n == 1 && method == HTTP_GET) {
        JsonDocument d;
        JsonArray arr = d["pumps"].to<JsonArray>();
        for (const auto& s : g_profiles.list()) {
            JsonObject o = arr.add<JsonObject>();
            o["id"] = s.id;
            o["name"] = s.name;
            o["pumpType"] = s.pumpType;
            o["driverType"] = driverTypeId(s.driver);
            o["family"] = familyOf(s.driver) == DriverFamily::DC ? "dc" : "stepper";
            o["calibrations"] = s.calibrations;
            o["activeCalibration"] = s.activeCalibration;
        }
        d["ok"] = true;
        sendDoc(r, 200, d);
        return;
    }
    if (n == 1 && method == HTTP_POST) {   // create / import
        PumpProfile* p = new PumpProfile;
        String err;
        if (!profileFromJson(body.as<JsonObjectConst>(), *p, err)) {
            delete p;
            sendError(r, 400, err);
            return;
        }
        String id = body["id"] | "";
        if (!ProfileStore::validId(id) || g_profiles.exists(id)) id = g_profiles.uniqueId(p->name.c_str());
        p->id = id.c_str();
        app::Job* j = new app::Job(app::JobType::SAVE_PROFILE);
        j->profile = p;
        if (!app::enqueue(j)) {
            sendError(r, 503, "busy");
            return;
        }
        JsonDocument d;
        d["ok"] = true;
        d["id"] = id;
        sendDoc(r, 201, d);
        return;
    }
    if (n < 2) {
        sendError(r, 404, "not found");
        return;
    }
    const String id = seg[1];
    if (!ProfileStore::validId(id) || !g_profiles.exists(id)) {
        sendError(r, 404, "unknown profile");
        return;
    }
    if (n == 2 && method == HTTP_GET) {
        AsyncWebServerResponse* resp = r->beginResponse(LittleFS, ProfileStore::pathFor(id), "application/json");
        resp->addHeader("Cache-Control", "no-store");
        r->send(resp);
        return;
    }
    if (n == 2 && method == HTTP_PUT) {
        std::unique_ptr<PumpProfile> p(new PumpProfile);
        String err;
        if (!profileFromJson(body.as<JsonObjectConst>(), *p, err)) {
            sendError(r, 400, err);
            return;
        }
        p->id = id.c_str();
        {
            LOCK_OR_503(r);
            for (int i = 0; i < MAX_CHANNELS; ++i) {
                Channel& c = app::channels[i];
                if (!c.hasProfile() || c.profile().id != p->id) continue;
                if (familyOf(p->motor.driverType) != familyOf(app::hardware.ch[i].driver)) {
                            sendError(r, 409, "profile is in use on a channel with a different motor type");
                    return;
                }
                // Motor/limit changes while running are applied live (the
                // driver clamps immediately); keep calibrations made since
                // the browser loaded the profile.
                if (p->calibrations.size() < c.profile().calibrations.size() && body["calibrations"].isNull())
                    p->calibrations = c.profile().calibrations;
            }
        }
        app::Job* j = new app::Job(app::JobType::SAVE_PROFILE);
        j->profile = p.release();
        if (!app::enqueue(j)) {
            sendError(r, 503, "busy");
            return;
        }
        sendOk(r);
        return;
    }
    if (n == 2 && method == HTTP_DELETE) {
        {
            LOCK_OR_503(r);
            for (auto& c : app::channels) {
                if (c.hasProfile() && String(c.profile().id.c_str()) == id) {
                    sendError(r, 409, "profile is assigned to a channel");
                    return;
                }
            }
        }
        app::Job* j = new app::Job(app::JobType::DELETE_PROFILE);
        j->id = id;
        app::enqueue(j);
        sendOk(r);
        return;
    }
    if (n == 3 && seg[2] == "duplicate" && method == HTTP_POST) {
        PumpProfile* p = new PumpProfile;
        if (!g_profiles.load(id, *p)) {
            delete p;
            sendError(r, 500, "cannot read profile");
            return;
        }
        String name = body["name"] | String(String(p->name.c_str()) + " (copy)");
        if (name.length() > 63) name = name.substring(0, 63);
        p->name = name.c_str();
        const String nid = g_profiles.uniqueId(name);
        p->id = nid.c_str();
        app::Job* j = new app::Job(app::JobType::SAVE_PROFILE);
        j->profile = p;
        app::enqueue(j);
        JsonDocument d;
        d["ok"] = true;
        d["id"] = nid;
        sendDoc(r, 201, d);
        return;
    }
    sendError(r, 405, "method not allowed");
}

void handleHardware(AsyncWebServerRequest* r, JsonDocument& body) {
    if (r->method() == HTTP_GET) {
        JsonDocument d;
        d["ok"] = true;
        LOCK_OR_503(r);
        JsonArray arr = d["channels"].to<JsonArray>();
        for (int i = 0; i < MAX_CHANNELS; ++i) {
            JsonObject o = arr.add<JsonObject>();
            hwToJson(app::hardware.ch[i], o);
            o["profile"] = app::channels[i].hasProfile() ? app::channels[i].profile().id.c_str() : "";
        }
        JsonArray drivers = d["driverTypes"].to<JsonArray>();
        const DriverType all[] = {DriverType::ULN2003, DriverType::STEP_DIR, DriverType::DRV8871, DriverType::PWM_DIR};
        for (DriverType t : all) {
            JsonObject o = drivers.add<JsonObject>();
            o["id"] = driverTypeId(t);
            o["family"] = familyOf(t) == DriverFamily::DC ? "dc" : "stepper";
            o["commandUnit"] = commandUnitFor(t);
            JsonArray lab = o["pinLabels"].to<JsonArray>();
            for (int s = 0; s < maxPinCount(t); ++s) lab.add(pinLabel(t, s));
            o["requiredPins"] = requiredPinCount(t);
            JsonArray defs = o["defaults"].to<JsonArray>();
            for (int i = 0; i < MAX_CHANNELS; ++i) {
                ChannelHw h;
                applyDefaultPins(h, i, t);
                JsonArray p = defs.add<JsonArray>();
                for (int s = 0; s < maxPinCount(t); ++s) p.add(h.pins[s]);
            }
        }
        sendDoc(r, 200, d);
        return;
    }
    if (r->method() != HTTP_PUT && r->method() != HTTP_POST) {
        sendError(r, 405, "method not allowed");
        return;
    }
    HardwareConfig hw;
    String profiles[MAX_CHANNELS];
    {
        LOCK_OR_503(r);
        hw = app::hardware;
    }
    JsonArrayConst chs = body["channels"];
    if (chs.isNull() || static_cast<int>(chs.size()) != MAX_CHANNELS) {
        sendError(r, 400, "expected channels[] with one entry per channel");
        return;
    }
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        JsonObjectConst c = chs[i];
        ChannelHw& h = hw.ch[i];
        h.driver = driverTypeFromId(c["driver"] | driverTypeId(h.driver));
        if (h.driver == DriverType::NONE) {
            sendError(r, 400, String("Ch") + (i + 1) + ": unknown driver type");
            return;
        }
        JsonArrayConst pins = c["pins"];
        for (int s = 0; s < 4; ++s) h.pins[s] = s < static_cast<int>(pins.size()) && s < maxPinCount(h.driver) ? (pins[s] | -1) : -1;
        h.enActiveLow = c["enActiveLow"] | true;
        h.sensor = strcmp(c["sensor"] | "slf3s", "none") == 0 ? SensorType::NONE : SensorType::SLF3S;
        h.pressure = strcmp(c["pressure"] | (h.pressure == PressureType::MPRLS ? "mprls" : "none"), "mprls") == 0 ? PressureType::MPRLS
                                                                                                             : PressureType::NONE;
        h.sda = c["sda"] | h.sda;
        h.scl = c["scl"] | h.scl;
        profiles[i] = c["profile"] | "";
    }
    PinError errs[12];
    const int ne = validateHardware(hw, errs, 12);
    if (ne > 0) {
        JsonDocument d;
        d["ok"] = false;
        d["error"] = "invalid pin configuration";
        JsonArray arr = d["pinErrors"].to<JsonArray>();
        for (int i = 0; i < ne && i < 12; ++i) {
            JsonObject e = arr.add<JsonObject>();
            e["channel"] = errs[i].channel + 1;
            e["field"] = errs[i].field;
            e["gpio"] = errs[i].gpio;
            e["reason"] = errs[i].reason;
        }
        sendDoc(r, 400, d);
        return;
    }
    // Profile compatibility with the (new) driver family.
    const auto summaries = g_profiles.list();
    DriverType pdrv[MAX_CHANNELS];
    {
        LOCK_OR_503(r);
        for (int i = 0; i < MAX_CHANNELS; ++i) {
            if (!app::channels[i].isIdle()) {
                sendError(r, 409, "stop all channels before changing hardware");
                return;
            }
            pdrv[i] = app::channels[i].hasProfile() ? app::channels[i].profile().motor.driverType : DriverType::NONE;
            if (profiles[i].isEmpty() && app::channels[i].hasProfile()) profiles[i] = app::channels[i].profile().id.c_str();
        }
    }
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        for (const auto& s : summaries)
            if (s.id == profiles[i]) pdrv[i] = s.driver;
        if (!profiles[i].isEmpty() && familyOf(pdrv[i]) != familyOf(hw.ch[i].driver)) {
            sendError(r, 409, String("Ch") + (i + 1) + ": selected pump profile is for a different motor type; pick a matching profile");
            return;
        }
    }
    app::requestHardware(hw);
    {
        LOCK_OR_503(r);
        app::hardware = hw;   // reported immediately; rtTask applies drivers
    }
    app::enqueue(new app::Job(app::JobType::SAVE_HARDWARE));
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        if (profiles[i].isEmpty()) continue;
        app::Job* j = new app::Job(app::JobType::ASSIGN_PROFILE);
        j->ch = i;
        j->id = profiles[i];
        app::enqueue(j);
    }
    sendOk(r);
}

void handleConfig(AsyncWebServerRequest* r, JsonDocument& body) {
    if (r->method() == HTTP_GET) {
        JsonDocument d;
        d["ok"] = true;
        LOCK_OR_503(r);
        settingsToJson(d["config"].to<JsonObject>());
        sendDoc(r, 200, d);
        return;
    }
    bool wifiChanged = false;
    {
        LOCK_OR_503(r);
        AppSettings s = app::settings;
        if (!body["watchdogS"].isNull()) s.watchdogS = body["watchdogS"].as<float>();
        if (!body["telemetryHz"].isNull()) s.telemetryHz = body["telemetryHz"].as<float>();
        if (!body["flowUnit"].isNull()) s.flowUnit = strcmp(body["flowUnit"] | "", "mL/min") == 0 ? 1 : 0;
        if (!body["pressureUnit"].isNull()) {
            int u = -1;
            for (int k = 0; k < 4; ++k)
                if (!strcmp(body["pressureUnit"] | "", kPressureUnits[k])) u = k;
            if (u < 0) {
                sendError(r, 400, "pressureUnit must be kPa, psi, mmHg or cmH2O");
                return;
            }
            s.pressureUnit = static_cast<uint8_t>(u);
        }
        if (!body["sensorFaultAction"].isNull()) s.sensorFaultStops = strcmp(body["sensorFaultAction"] | "", "stop") == 0;
        if (!(s.watchdogS >= 2 && s.watchdogS <= 600)) {
            sendError(r, 400, "watchdogS must be 2-600");
            return;
        }
        if (!(s.telemetryHz >= 0.5f && s.telemetryHz <= 20)) {
            sendError(r, 400, "telemetryHz must be 0.5-20");
            return;
        }
        JsonVariantConst w = body["wifi"];
        if (!w.isNull()) {
            if (!w["ssid"].isNull()) {
                strlcpy(s.staSsid, w["ssid"] | "", sizeof(s.staSsid));
                wifiChanged = true;
            }
            if (!w["password"].isNull()) {
                strlcpy(s.staPass, w["password"] | "", sizeof(s.staPass));
                wifiChanged = true;
            }
        }
        if (!body["apPassword"].isNull()) {
            const String ap = body["apPassword"] | "";
            if (ap.length() < 8 || ap.length() > 63) {
                sendError(r, 400, "AP password must be 8-63 characters (applies after reboot)");
                return;
            }
            strlcpy(s.apPass, ap.c_str(), sizeof(s.apPass));
        }
        app::settings = s;
        for (auto& c : app::channels) c.setSensorFaultStops(s.sensorFaultStops);
    }
    app::enqueue(new app::Job(app::JobType::SAVE_SETTINGS));
    if (wifiChanged) app::enqueue(new app::Job(app::JobType::APPLY_WIFI));
    sendOk(r);
}

void handleCalibration(AsyncWebServerRequest* r, int ch, const String& action, JsonDocument& body) {
    const uint32_t now = millis();
    if (action == "status") {
        JsonDocument d;
        d["ok"] = true;
        LOCK_OR_503(r);
        app::channels[ch].calStatusJson(d["status"].to<JsonObject>(), now);
        d["state"] = channelStateName(app::channels[ch].isIdle() ? ChannelState::IDLE : ChannelState::CALIBRATING);
        sendDoc(r, 200, d);
        return;
    }
    if (r->method() != HTTP_POST) {
        sendError(r, 405, "use POST");
        return;
    }
    String err;
    if (action == "start") {
        SweepConfig cfg;
        float maxCmd;
        {
            LOCK_OR_503(r);
            maxCmd = app::channels[ch].profile().motor.maxCommand;
        }
        if (!parseSweep(body, cfg, err, maxCmd)) {
            sendError(r, 400, err);
            return;
        }
        LOCK_OR_503(r);
        if (app::estopLatched) {
            sendError(r, 409, "E-STOP is latched");
            return;
        }
        if (const char* e = app::channels[ch].calStart(cfg, body["keepRunning"] | true, now)) {
            sendError(r, 409, e);
            return;
        }
        app::watchdog.feed(now);
        sendOk(r);
        return;
    }
    if (action == "abort") {
        LOCK_OR_503(r);
        app::channels[ch].calAbort(now);
        sendOk(r);
        return;
    }
    if (action == "discard") {
        LOCK_OR_503(r);
        app::channels[ch].calDiscard();
        sendOk(r);
        return;
    }
    if (action == "point") {
        const int idx = body["index"] | -1;
        float flow = numOr(body["flow_uLmin"], NAN);
        const float vol = numOr(body["volume_uL"], NAN);
        const float dur = numOr(body["duration_s"], NAN);
        if (std::isnan(flow) && vol > 0 && dur > 0) flow = vol / dur * 60.0f;
        const bool clear = body["clear"] | false;
        if (!clear && !std::isfinite(flow)) {
            sendError(r, 400, "give flow_uLmin, or volume_uL + duration_s");
            return;
        }
        LOCK_OR_503(r);
        if (!app::channels[ch].calReference(idx, flow, clear)) {
            sendError(r, 409, "point not measured yet (or bad index)");
            return;
        }
        sendOk(r);
        return;
    }
    if (action == "retry") {
        LOCK_OR_503(r);
        if (const char* e = app::channels[ch].calRetry(body["index"] | -1, body["keepRunning"] | true, now)) {
            sendError(r, 409, e);
            return;
        }
        app::watchdog.feed(now);
        sendOk(r);
        return;
    }
    if (action == "accept") {
        FitType ft = FitType::LINEAR;
        if (!fitTypeFromId(body["fitType"] | "linear", ft)) {
            sendError(r, 400, "fitType must be linear, quadratic or lookup");
            return;
        }
        std::unique_ptr<PumpProfile> copy(new PumpProfile);
        {
            LOCK_OR_503(r);
            Channel& c = app::channels[ch];
            if (!c.calAccept(ft, body["name"] | "", body["tubingNote"] | "", app::epochS(), err)) {
                sendError(r, 409, err);
                return;
            }
            *copy = c.profile();
        }
        app::Job* j = new app::Job(app::JobType::SAVE_PROFILE);
        j->profile = copy.release();
        app::enqueue(j);
        app::logEvent(ch, 0, "Calibration accepted (%s) and set active", fitTypeId(ft));
        sendOk(r);
        return;
    }
    sendError(r, 404, "unknown calibration action");
}

// Streams the history ring buffer as JSON rows (chunked, no big allocation).
void handleHistory(AsyncWebServerRequest* r) {
    int ch = -1;
    if (const AsyncWebParameter* p = r->getParam("ch")) ch = parseChannel(p->value());
    if (ch < 0) {
        sendError(r, 400, "ch=1..N required");
        return;
    }
    struct Cursor {
        size_t i = 0;
        size_t n = 0;
        int ch = 0;
        int stage = 0;
        int64_t off = 0;
    };
    auto cur = std::make_shared<Cursor>();
    cur->n = app::historySize();
    cur->ch = ch;
    cur->off = app::uptimeToEpochOffsetMs();
    r->sendChunked("application/json", [cur](uint8_t* buf, size_t max, size_t) -> size_t {
        size_t w = 0;
        auto put = [&](const char* s) {
            const size_t l = strlen(s);
            if (w + l > max) return false;
            memcpy(buf + w, s, l);
            w += l;
            return true;
        };
        if (cur->stage == 0) {
            char hdr[160];
            snprintf(hdr, sizeof(hdr), "{\"ok\":true,\"ch\":%d,\"synced\":%s,\"cols\":[\"t\",\"flow\",\"setpoint\",\"command\",\"temp\",\"state\",\"pressure\"],\"rows\":[",
                     cur->ch + 1, cur->off ? "true" : "false");
            if (!put(hdr)) return 0;
            cur->stage = 1;
        }
        char row[128];
        auto fmt = [](char* o, size_t n, float v) {
            if (std::isfinite(v)) snprintf(o, n, "%.3f", v);
            else snprintf(o, n, "null");
        };
        while (cur->stage == 1 && cur->i < cur->n) {
            app::HistoryEntry e;
            if (!app::historyAt(cur->i, e)) {
                cur->i = cur->n;
                break;
            }
            const HistoryChannel& h = e.ch[cur->ch];
            char f[16], sp[16], c[16], t[16], p[16];
            fmt(f, sizeof(f), h.flow);
            fmt(sp, sizeof(sp), h.setpoint);
            fmt(c, sizeof(c), h.command);
            fmt(t, sizeof(t), h.tempCc == INT16_MIN ? NAN : h.tempCc / 100.0f);
            fmt(p, sizeof(p), h.pressureCkPa == INT16_MIN ? NAN : h.pressureCkPa / 100.0f);
            snprintf(row, sizeof(row), "%s[%lld,%s,%s,%s,%s,%u,%s]", cur->i ? "," : "",
                     static_cast<long long>(e.tMs + cur->off), f, sp, c, t, h.state, p);
            if (!put(row)) return w;
            ++cur->i;
        }
        if (cur->stage == 1) {
            if (!put("]}")) return w;
            cur->stage = 2;
        }
        return w;
    });
}

// Both channels' history as CSV.
void handleLogCsv(AsyncWebServerRequest* r) {
    struct Cursor {
        size_t i = 0, n = 0;
        int stage = 0;
        int64_t off = 0;
    };
    auto cur = std::make_shared<Cursor>();
    cur->n = app::historySize();
    cur->off = app::uptimeToEpochOffsetMs();
    AsyncWebServerResponse* resp = r->beginChunkedResponse("text/csv", [cur](uint8_t* buf, size_t max, size_t) -> size_t {
        size_t w = 0;
        auto put = [&](const char* s) {
            const size_t l = strlen(s);
            if (w + l > max) return false;
            memcpy(buf + w, s, l);
            w += l;
            return true;
        };
        if (cur->stage == 0) {
            String h = "time_ms,iso_time";
            for (int c = 1; c <= MAX_CHANNELS; ++c) {
                h += ",ch" + String(c) + "_flow_uLmin,ch" + String(c) + "_setpoint_uLmin,ch" + String(c) + "_command,ch" + String(c) +
                     "_temp_C,ch" + String(c) + "_pressure_kPa,ch" + String(c) + "_state,ch" + String(c) + "_alarms";
            }
            h += "\n";
            if (!put(h.c_str())) return 0;
            cur->stage = 1;
        }
        while (cur->i < cur->n) {
            app::HistoryEntry e;
            if (!app::historyAt(cur->i, e)) {
                cur->i = cur->n;
                break;
            }
            char line[96 + 96 * MAX_CHANNELS];
            const long long t = static_cast<long long>(e.tMs + cur->off);
            int k = snprintf(line, sizeof(line), "%lld,", t);
            if (cur->off) {
                const time_t s = static_cast<time_t>(t / 1000);
                struct tm tmv;
                gmtime_r(&s, &tmv);
                k += strftime(line + k, sizeof(line) - k, "%Y-%m-%dT%H:%M:%SZ", &tmv);
            }
            for (int c = 0; c < MAX_CHANNELS; ++c) {
                const HistoryChannel& h = e.ch[c];
                auto f = [](float v, char* o) {
                    if (std::isfinite(v)) snprintf(o, 16, "%.3f", v);
                    else o[0] = 0;
                };
                char a[16], b[16], cc[16], d[16], pp[16];
                f(h.flow, a);
                f(h.setpoint, b);
                f(h.command, cc);
                f(h.tempCc == INT16_MIN ? NAN : h.tempCc / 100.0f, d);
                f(h.pressureCkPa == INT16_MIN ? NAN : h.pressureCkPa / 100.0f, pp);
                k += snprintf(line + k, sizeof(line) - k, ",%s,%s,%s,%s,%s,%s,%u", a, b, cc, d, pp,
                              channelStateName(static_cast<ChannelState>(h.state)), h.flags);
            }
            line[k++] = '\n';
            line[k] = 0;
            if (!put(line)) return w;
            ++cur->i;
        }
        return w;
    });
    resp->addHeader("Content-Disposition", "attachment; filename=\"biopump-history.csv\"");
    r->send(resp);
}

void handleSensors(AsyncWebServerRequest* r, int ch, const String& action, JsonDocument& body) {
    const uint32_t now = millis();
    if (action == "pressure-zero" && r->method() == HTTP_POST) {
        LOCK_OR_503(r);
        Channel& c = app::channels[ch];
        if (!c.hasPressure()) {
            sendError(r, 409, "no pressure sensor on this channel");
            return;
        }
        float v = NAN;
        if (!(body["clear"] | false)) {
            if (body["offset_kPa"].isNull()) {   // measure
                if (const char* e = c.pressureTare(body["seconds"] | 5.0f, now)) {
                    sendError(r, 409, e);
                    return;
                }
                sendOk(r);
                return;
            }
            v = body["offset_kPa"].as<float>();   // manual atmosphere value (absolute kPa)
            if (!std::isfinite(v) || v < 50 || v > 120) {
                sendError(r, 400, "offset_kPa is the absolute atmospheric pressure, 50-120 kPa");
                return;
            }
        }
        c.setPressureZero(v);
        app::Job* j = new app::Job(app::JobType::SAVE_PRESSURE_ZERO);
        j->ch = ch;
        j->value = v;
        app::enqueue(j);
        sendOk(r);
        return;
    }
    if (action == "pressure-alarm" && r->method() == HTTP_POST) {
        const float v = body["kPa"] | -1.0f;
        if (!std::isfinite(v) || v < 0 || v > 160) {
            sendError(r, 400, "kPa must be 0 (off) to 160 (gauge)");
            return;
        }
        LOCK_OR_503(r);
        Channel& c = app::channels[ch];
        if (v > 0 && !c.hasPressure()) {
            sendError(r, 409, "no pressure sensor on this channel");
            return;
        }
        c.setPressureAlarm(v);
        app::Job* j = new app::Job(app::JobType::SAVE_PRESSURE_ALARM);
        j->ch = ch;
        j->value = v;
        app::enqueue(j);
        if (v > 0) app::logEvent(ch, 0, "Pressure alarm limit set to %.1f kPa (gauge)", v);
        else app::logEvent(ch, 0, "Pressure alarm off");
        sendOk(r);
        return;
    }
    if (action == "zero" && r->method() == HTTP_POST) {
        LOCK_OR_503(r);
        Channel& c = app::channels[ch];
        if (body["clear"] | false) {
            c.setZeroOffset(0);
            app::Job* j = new app::Job(app::JobType::SAVE_ZERO);
            j->ch = ch;
            j->value = 0;
            app::enqueue(j);
            sendOk(r);
            return;
        }
        if (!body["offset_uLmin"].isNull()) {   // manual offset entry
            const float v = body["offset_uLmin"].as<float>();
            if (!std::isfinite(v) || std::fabs(v) > 5000) {
                sendError(r, 400, "offset out of range");
                return;
            }
            c.setZeroOffset(v);
            app::Job* j = new app::Job(app::JobType::SAVE_ZERO);
            j->ch = ch;
            j->value = v;
            app::enqueue(j);
            sendOk(r);
            return;
        }
        if (const char* e = c.zero(body["seconds"] | 10.0f, now)) {
            sendError(r, 409, e);
            return;
        }
        sendOk(r);
        return;
    }
    if (action.isEmpty() && r->method() == HTTP_GET) {
        JsonDocument d;
        d["ok"] = true;
        LOCK_OR_503(r);
        ChannelSnapshot s;
        app::channels[ch].snapshot(s, now);
        JsonObject o = d["sensor"].to<JsonObject>();
        o["type"] = app::hardware.ch[ch].sensor == SensorType::SLF3S ? "SLF3S-1300F" : "none";
        o["status"] = sensorStatusName(s.sensorStatus);
        o["flow_uLmin"] = s.flow;
        o["flow_avg_uLmin"] = s.flowAvg;
        o["flow_std"] = s.flowStd;
        o["temp_C"] = s.tempC;
        o["flags"] = s.flags;
        o["zeroOffset_uLmin"] = s.zeroOffset;
        o["zeroing"] = s.zeroing;
        o["windowS"] = s.filterWindowS;
        JsonObject e = o["errors"].to<JsonObject>();
        e["crc"] = s.errors.crc;
        e["i2c"] = s.errors.i2c;
        e["consecutive"] = s.errors.consecutive;
        e["reinits"] = s.errors.reinits;
        JsonObject po = d["pressure"].to<JsonObject>();
        po["type"] = s.pressurePresent ? "MPRLS0025PA00001A" : "none";
        pressureToJson(s, po);
        sendDoc(r, 200, d);
        return;
    }
    sendError(r, 404, "not found");
}

void handleSim(AsyncWebServerRequest* r, int ch, JsonDocument& body) {
#ifdef SIM_MODE
    LOCK_OR_503(r);
    SimPlant& p = app::channels[ch].plant();
    if (!body["bubble"].isNull()) p.injectBubble(body["bubble"] | 3.0f);
    if (!body["fault"].isNull()) p.setFault(body["fault"] | false);
    if (!body["occlusion"].isNull()) p.setOcclusion(body["occlusion"] | false);
    if (!body["pressureFault"].isNull()) p.setPressureFault(body["pressureFault"] | false);
    if (!body["gainScale"].isNull()) {
        SimPlantParams q = p.params();
        q.gain *= body["gainScale"] | 1.0f;
        p.setParams(q);
    }
    JsonDocument d;
    d["ok"] = true;
    d["gain"] = p.params().gain;
    d["fault"] = p.fault();
    d["bubble"] = p.bubbleActive();
    d["occlusion"] = p.occluded();
    d["pressureFault"] = p.pressureFault();
    sendDoc(r, 200, d);
#else
    sendError(r, 404, "simulation endpoints exist only in the esp32dev_sim build");
#endif
}

void dispatch(AsyncWebServerRequest* r) {
    String seg[6];
    const int n = splitPath(r->url(), seg, 6);
    JsonDocument body;
    if (r->_tempObject) {
        DeserializationError e = deserializeJson(body, static_cast<const char*>(r->_tempObject));
        if (e) {
            sendError(r, 400, String("invalid JSON: ") + e.c_str());
            return;
        }
    } else if (r->contentLength() > kMaxBody) {
        sendError(r, 413, "body too large");
        return;
    }
    const auto method = r->method();
    if (method == HTTP_OPTIONS) {   // CORS preflight
        r->send(204);
        return;
    }
    if (n == 0) {
        sendError(r, 404, "see docs/API.md");
        return;
    }
    const String& s0 = seg[0];

    if (s0 == "state" && method == HTTP_GET) {
        JsonDocument d;
        buildState(d);
        sendDoc(r, 200, d);
        return;
    }
    if (s0 == "hb") {
        app::watchdog.feed(millis());
        sendOk(r);
        return;
    }
    if (s0 == "time" && method == HTTP_POST) {
        const double e = body["epochMs"] | 0.0;
        if (e < 1.6e12) {
            sendError(r, 400, "epochMs required");
            return;
        }
        app::setEpochMs(static_cast<uint64_t>(e));
        sendOk(r);
        return;
    }
    if (s0 == "config") return handleConfig(r, body);
    if (s0 == "hardware") return handleHardware(r, body);
    if (s0 == "pumps") return handlePumps(r, seg, n, body);
    if (s0 == "estop" && method == HTTP_POST) {
        if (n >= 2 && seg[1] == "reset") app::resetEstop();
        else app::estopAll("API");
        sendOk(r);
        return;
    }
    if (s0 == "history" && method == HTTP_GET) return handleHistory(r);
    if (s0 == "log.csv" && method == HTTP_GET) return handleLogCsv(r);
    if (s0 == "events" && method == HTTP_GET) {
        static app::EventRec ev[48];
        const int k = app::eventsSince(0, ev, 48);
        JsonDocument d;
        d["ok"] = true;
        JsonArray arr = d["events"].to<JsonArray>();
        for (int i = 0; i < k; ++i) {
            JsonObject o = arr.add<JsonObject>();
            o["seq"] = ev[i].seq;
            o["t"] = ev[i].tMs;
            o["ch"] = ev[i].ch >= 0 ? ev[i].ch + 1 : 0;
            o["level"] = ev[i].level;
            o["msg"] = ev[i].msg;
        }
        sendDoc(r, 200, d);
        return;
    }
    if (s0 == "history" && method == HTTP_DELETE) {
        app::historyClear();
        sendOk(r);
        return;
    }
    if (s0 == "reboot" && method == HTTP_POST) {
        {
            app::Lock l;
            for (auto& c : app::channels) {
                if (!c.isIdle()) {
                    sendError(r, 409, "stop all channels first");
                    return;
                }
            }
        }
        sendOk(r);
        app::logEvent(-1, 1, "Rebooting");
        // Give the response time to go out; motors are already idle.
        static TimerHandle_t t = xTimerCreate("reboot", pdMS_TO_TICKS(500), pdFALSE, nullptr, [](TimerHandle_t) { ESP.restart(); });
        xTimerStart(t, 0);
        return;
    }

    // Per-channel resources: /api/<res>/<n>/<action>
    if (n >= 2 && (s0 == "channels" || s0 == "calibration" || s0 == "sensors" || s0 == "sim")) {
        const int ch = parseChannel(seg[1]);
        if (ch < 0) {
            sendError(r, 404, "no such channel (use 1.." + String(MAX_CHANNELS) + ")");
            return;
        }
        const String action = n >= 3 ? seg[2] : String();
        if (s0 == "calibration") return handleCalibration(r, ch, action, body);
        if (s0 == "sensors") return handleSensors(r, ch, action, body);
        if (s0 == "sim") return handleSim(r, ch, body);
        if (method != HTTP_POST) {
            sendError(r, 405, "use POST");
            return;
        }
        int code = 200;
        if (const char* e = channelAction(ch, action.c_str(), body.as<JsonVariantConst>(), code)) {
            sendError(r, code, e);
            return;
        }
        sendOk(r);
        return;
    }
    sendError(r, 404, "unknown endpoint: " + r->url());
}

}  // namespace

void begin(AsyncWebServer& server, AsyncWebSocket& ws) {
    server.on(AsyncURIMatcher::prefix("/api"), HTTP_ANY, dispatch, nullptr, onBody);
    server.addHandler(&ws);
    server.serveStatic("/", LittleFS, "/www/").setDefaultFile("index.html").setCacheControl("no-cache");
    server.onNotFound([](AsyncWebServerRequest* r) {
        if (r->method() == HTTP_OPTIONS) {
            r->send(204);
            return;
        }
        r->send(404, "text/plain", "Not found. The web UI lives at / (did you run 'pio run -t uploadfs'?)");
    });
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
}

}  // namespace api
