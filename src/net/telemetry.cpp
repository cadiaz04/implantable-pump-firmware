// WebSocket /ws: telemetry push (server → client) and control messages
// (client → server). See docs/API.md.
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "app/app.h"
#include "net/api.h"
#include "sensors/sensor_registry.h"

namespace api {
namespace {

AsyncWebSocket* g_ws = nullptr;

void reply(AsyncWebSocketClient* c, JsonVariantConst id, const char* err) {
    JsonDocument d;
    d["type"] = "ack";
    d["id"] = id;
    d["ok"] = err == nullptr;
    if (err) d["error"] = err;
    String s;
    serializeJson(d, s);
    c->text(s);
}

void handleMessage(AsyncWebSocketClient* c, const char* data, size_t len) {
    JsonDocument d;
    if (deserializeJson(d, data, len)) return;
    const char* type = d["type"] | "";
    const uint32_t now = millis();
    if (!strcmp(type, "hb")) {
        app::watchdog.feed(now);
        return;
    }
    if (!strcmp(type, "time")) {
        const double e = d["epochMs"] | 0.0;
        if (e > 1.6e12) app::setEpochMs(static_cast<uint64_t>(e));
        app::watchdog.feed(now);
        return;
    }
    if (!strcmp(type, "estop")) {
        app::estopAll(("WebSocket client " + String(c->id())).c_str());
        reply(c, d["id"], nullptr);
        return;
    }
    if (!strcmp(type, "estop_reset")) {
        app::resetEstop();
        reply(c, d["id"], nullptr);
        return;
    }
    // Channel actions: {"type":"run"|"stop"|"pause"|"resume"|"profile"|"volume-reset", "ch":1, ...}
    const int ch = (d["ch"] | 0) - 1;
    int code = 0;
    const char* err = channelAction(ch, type, d.as<JsonVariantConst>(), code);
    reply(c, d["id"], err);
}

void onEvent(AsyncWebSocket*, AsyncWebSocketClient* c, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        JsonDocument d;
        d["type"] = "hello";
        d["sim"] = app::simMode();
        d["version"] = app::firmwareVersion();
        d["clientId"] = c->id();
        String s;
        serializeJson(d, s);
        c->text(s);
        app::watchdog.feed(millis());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            handleMessage(c, reinterpret_cast<const char*>(data), len);
        }
    }
}

void telemetryTask(void*) {
    uint32_t lastSeq = app::lastEventSeq();
    static app::EventRec ev[16];
    ChannelSnapshot snaps[MAX_CHANNELS];
    for (;;) {
        float hz;
        {
            app::Lock l;
            hz = app::settings.telemetryHz;
        }
        vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(1000.0f / hz)));
        g_ws->cleanupClients(DEFAULT_MAX_WS_CLIENTS);

        // Events first (they're rare and important).
        int k;
        while ((k = app::eventsSince(lastSeq, ev, 16)) > 0) {
            for (int i = 0; i < k; ++i) {
                lastSeq = ev[i].seq;
                if (g_ws->count() == 0) continue;
                JsonDocument d;
                d["type"] = "event";
                d["seq"] = ev[i].seq;
                d["t"] = ev[i].tMs;
                d["ch"] = ev[i].ch >= 0 ? ev[i].ch + 1 : 0;
                d["level"] = ev[i].level;
                d["msg"] = ev[i].msg;
                String s;
                serializeJson(d, s);
                g_ws->textAll(s);
            }
        }
        if (g_ws->count() == 0) continue;

        const uint32_t now = millis();
        bool estop;
        float hbAge;
        {
            app::Lock l(50);
            if (!l.ok) continue;
            for (int i = 0; i < MAX_CHANNELS; ++i) app::channels[i].snapshot(snaps[i], now);
            estop = app::estopLatched;
            hbAge = app::watchdog.ageS(now);
        }
        JsonDocument d;
        d["type"] = "tel";
        d["t"] = app::epochMs();
        d["up"] = now;
        d["synced"] = app::timeSynced();
        d["estop"] = estop;
        d["hbAge"] = hbAge;
        JsonArray chs = d["ch"].to<JsonArray>();
        JsonObject sig = d["sig"].to<JsonObject>();
        for (int i = 0; i < MAX_CHANNELS; ++i) {
            const ChannelSnapshot& s = snaps[i];
            JsonObject o = chs.add<JsonObject>();
            o["state"] = channelStateName(s.state);
            o["mode"] = runModeName(s.mode);
            o["command"] = s.command;
            o["commandActual"] = s.commandActual;
            o["commandUnit"] = commandUnitFor(s.driver);
            o["stepRate_Hz"] = s.stepRateHz;
            o["setpoint"] = s.setpoint;
            o["manualCommand"] = s.manualCommand;
            o["reverse"] = s.reverse;
            o["flow_uLmin"] = s.flow;
            o["flow_avg_uLmin"] = s.flowAvg;
            o["flow_std"] = s.flowStd;
            o["temp_C"] = s.tempC;
            o["volume_uL"] = s.volumeUl;
            o["volumeEstimated"] = s.volumeEstimated;
            o["runVolume_uL"] = s.runVolumeUl;
            o["plannedVolume_uL"] = s.plannedVolumeUl;
            o["targetVolume_uL"] = s.targetVolumeUl;
            o["eta_s"] = s.etaS;
            o["elapsed_s"] = s.runElapsedS;
            o["flags"] = s.flags;
            o["alarms"] = s.alarms;
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
            JsonObject cal = o["cal"].to<JsonObject>();
            cal["phase"] = sweepPhaseName(s.calPhase);
            cal["index"] = s.calIndex;
            cal["points"] = s.calPoints;
            cal["elapsed"] = s.calPhaseElapsed;
            cal["duration"] = s.calPhaseDuration;
            if (s.pressurePresent) {
                o["pressure_kPa"] = s.pressureKpa;
                o["pressureAbs_kPa"] = s.pressureAbsKpa;
                o["pressureTared"] = std::isfinite(s.pressureZeroKpa);
                o["pressureZero_kPa"] = s.pressureZeroKpa;
                o["pressureSensor"] = sensorStatusName(s.pressureStatus);
                o["pressureSaturated"] = s.pressureSaturated;
                o["pressureTaring"] = s.pressureTaring;
                o["pressureTareProgress"] = s.pressureTareProgress;
                o["pressureAlarm_kPa"] = s.pressureAlarmKpa;
                JsonObject pe = o["pressureErrors"].to<JsonObject>();
                pe["frame"] = s.pressureErrors.crc;
                pe["i2c"] = s.pressureErrors.i2c;
                pe["reinits"] = s.pressureErrors.reinits;
            }

            const String n(i + 1);
            sig[String("f") + n] = s.flow;
            sig[String("fa") + n] = s.flowAvg;
            sig[String("sp") + n] = s.setpoint;
            sig[String("c") + n] = s.commandActual;
            sig[String("t") + n] = s.tempC;
            sig[String("v") + n] = s.volumeUl;
            if (s.pressurePresent) sig[String("p") + n] = s.pressureKpa;
        }
        for (int k2 = 0; k2 < g_sensors.count(); ++k2) {
            Sensor* sn = g_sensors.at(k2);
            for (int j = 0; j < sn->signalCount(); ++j) sig[sn->signalInfo(j).id] = sn->signalValue(j);
        }
        String out;
        serializeJson(d, out);
        g_ws->textAll(out);
    }
}

}  // namespace

void startTelemetryTask(AsyncWebSocket& ws) {
    g_ws = &ws;
    ws.onEvent(onEvent);
    xTaskCreatePinnedToCore(telemetryTask, "telemetry", 8192, nullptr, 2, nullptr, 0);
}

}  // namespace api
