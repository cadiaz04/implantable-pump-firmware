// =============================================================================
// BioPump Controller — ESP32 pump control + flow calibration
//
//   setup(): load settings → mount LittleFS / profiles → Wi-Fi (AP+STA) →
//            HTTP + WebSocket → start tasks. All motors are OFF at boot and
//            no run is ever resumed after a reboot.
//
//   core 1: rt task (sensors, run state machines, motor ramps) + step ISRs
//   core 0: Wi-Fi, AsyncTCP (HTTP/WS callbacks), telemetry, storage tasks
// =============================================================================
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "app/app.h"
#include "config.h"
#include "net/api.h"
#include "net/wifi_manager.h"
#include "pumps/profile_store.h"
#include "sensors/sensor_registry.h"

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

void setup() {
    Serial.begin(115200);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(200);
    Serial.printf("\n=== BioPump Controller %s%s ===\n", app::firmwareVersion(), app::simMode() ? " [SIMULATION]" : "");

    app::begin();
    g_profiles.begin();

    // Per-channel flow and pressure sensors are configured in Settings →
    // Hardware. Any OTHER sensor: implement Sensor and register it here, e.g.
    //   static MySensor s1(...);
    //   g_sensors.add(&s1);
    // Its signals then appear in telemetry, the live plots and CSV exports.

    wifi_manager::begin(app::settings);
    api::begin(server, ws);
    server.begin();
    api::startTelemetryTask(ws);
    app::startTasks();

    app::logEvent(-1, 0, "Boot complete%s. All motors stopped.", app::simMode() ? " (SIMULATION MODE)" : "");
    Serial.printf("Open http://%s/ (AP %s) or http://%s.local/\n", wifi_manager::apIp().c_str(), wifi_manager::apSsid().c_str(),
                  app::settings.hostname);
}

void loop() {
    // Everything runs in dedicated tasks; print a heartbeat line for the
    // serial log every 10 s.
    static uint32_t last = 0;
    if (millis() - last > 10000) {
        last = millis();
        String line;
        {
            app::Lock l(50);
            if (l.ok) {
                for (int i = 0; i < MAX_CHANNELS; ++i) {
                    ChannelSnapshot s;
                    app::channels[i].snapshot(s, millis());
                    line += String(" | Ch") + (i + 1) + " " + channelStateName(s.state) + " cmd=" + String(s.commandActual, 2) +
                            " flow=" + String(s.flowAvg, 1) + "uL/min";
                }
            }
        }
        Serial.printf("[status] heap=%u%s%s\n", ESP.getFreeHeap(), wifi_manager::staConnected() ? (" sta=" + wifi_manager::staIp()).c_str() : "",
                      line.c_str());
    }
    delay(100);
}
