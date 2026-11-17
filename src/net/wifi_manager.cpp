#include "wifi_manager.h"
#include <ESPmDNS.h>
#include <WiFi.h>

namespace wifi_manager {
namespace {
String g_apSsid;
}

void begin(const AppSettings& s) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);   // lower latency for the WebSocket
    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    char ssid[24];
    snprintf(ssid, sizeof(ssid), "BioPump-%02X%02X", mac[4], mac[5]);
    g_apSsid = ssid;
    WiFi.softAP(ssid, s.apPass);
    Serial.printf("[wifi] AP \"%s\" at %s\n", ssid, WiFi.softAPIP().toString().c_str());
    WiFi.setHostname(s.hostname);
    applyStation(s);
    if (MDNS.begin(s.hostname)) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[wifi] mDNS http://%s.local\n", s.hostname);
    }
}

void applyStation(const AppSettings& s) {
    WiFi.disconnect(false, false);
    if (s.staSsid[0]) {
        WiFi.setAutoReconnect(true);
        WiFi.begin(s.staSsid, s.staPass);
        Serial.printf("[wifi] joining \"%s\"\n", s.staSsid);
    }
}

String apSsid() { return g_apSsid; }
String apIp() { return WiFi.softAPIP().toString(); }
bool staConnected() { return WiFi.status() == WL_CONNECTED; }
String staIp() { return staConnected() ? WiFi.localIP().toString() : String(); }
int apClients() { return WiFi.softAPgetStationNum(); }

}  // namespace wifi_manager
