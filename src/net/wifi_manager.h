#pragma once
// Wi-Fi: always-on access point (BioPump-XXXX, 192.168.4.1) plus an optional
// station connection to an existing network, and mDNS (biopump.local).
#include <Arduino.h>
#include "config/settings_store.h"

namespace wifi_manager {
void begin(const AppSettings& s);
void applyStation(const AppSettings& s);   // re-join with new credentials
String apSsid();
String apIp();
bool staConnected();
String staIp();
int apClients();
}  // namespace wifi_manager
