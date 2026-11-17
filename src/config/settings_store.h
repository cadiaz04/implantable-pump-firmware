#pragma once
// Persistent settings in NVS (Preferences): hardware/pin config, app
// settings, per-channel profile assignment and zero offsets.
// Nothing persisted here can start a motor: runs are never saved.
#include <Arduino.h>
#include "config/hardware_config.h"

struct AppSettings {
    char staSsid[33] = "";
    char staPass[65] = "";
    char apPass[65] = "";
    char hostname[32] = "";
    float watchdogS = DEFAULT_WATCHDOG_TIMEOUT_S;
    float telemetryHz = DEFAULT_TELEMETRY_HZ;
    uint8_t flowUnit = 0;            // UI default: 0 µL/min, 1 mL/min
    uint8_t pressureUnit = 0;        // UI default: 0 kPa, 1 psi, 2 mmHg, 3 cmH2O (data is always kPa)
    bool sensorFaultStops = false;   // false: pause on sensor fault / air, true: stop
};

namespace settings_store {
void begin();
bool loadHardware(HardwareConfig& out);   // false → defaults returned
void saveHardware(const HardwareConfig& hw);
void loadSettings(AppSettings& out);
void saveSettings(const AppSettings& s);
String loadChannelProfile(int ch);
void saveChannelProfile(int ch, const String& id);
float loadZeroOffset(int ch);
void saveZeroOffset(int ch, float v);
// Pressure tare = absolute pressure with the line open to air (kPa).
// NAN = never tared.
float loadPressureZero(int ch);
void savePressureZero(int ch, float v);
// High-pressure (occlusion) alarm threshold, gauge kPa; 0 = off.
float loadPressureAlarm(int ch);
void savePressureAlarm(int ch, float v);
void factoryReset();
}  // namespace settings_store
