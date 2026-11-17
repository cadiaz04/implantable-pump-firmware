#include "settings_store.h"
#include <Preferences.h>
#include "config/pin_rules.h"

namespace settings_store {
namespace {
Preferences prefs;
constexpr uint8_t kHwVersion = 2;      // 2: per-channel pressure sensor
constexpr uint8_t kSettingsVersion = 2;  // 2: pressure unit

struct HwBlob {
    uint8_t version;
    uint8_t channels;
    HardwareConfig hw;
};
struct SettingsBlob {
    uint8_t version;
    AppSettings s;
};
String key(const char* base, int ch) { return String(base) + ch; }
}  // namespace

void begin() { prefs.begin("biopump", false); }

bool loadHardware(HardwareConfig& out) {
    HwBlob b;
    if (prefs.isKey("hw") && prefs.getBytesLength("hw") == sizeof(b) && prefs.getBytes("hw", &b, sizeof(b)) == sizeof(b) &&
        b.version == kHwVersion && b.channels == MAX_CHANNELS) {
        PinError errs[4];
        if (validateHardware(b.hw, errs, 4) == 0) {
            out = b.hw;
            return true;
        }
    }
    out = defaultHardwareConfig();
    return false;
}

void saveHardware(const HardwareConfig& hw) {
    HwBlob b;
    b.version = kHwVersion;
    b.channels = MAX_CHANNELS;
    b.hw = hw;
    prefs.putBytes("hw", &b, sizeof(b));
}

void loadSettings(AppSettings& out) {
    SettingsBlob b;
    out = AppSettings{};
    if (prefs.isKey("settings") && prefs.getBytesLength("settings") == sizeof(b) && prefs.getBytes("settings", &b, sizeof(b)) == sizeof(b) &&
        b.version == kSettingsVersion) {
        out = b.s;
    }
    if (strlen(out.apPass) < 8) strlcpy(out.apPass, DEFAULT_AP_PASSWORD, sizeof(out.apPass));
    if (out.hostname[0] == 0) strlcpy(out.hostname, DEFAULT_HOSTNAME, sizeof(out.hostname));
    if (!(out.watchdogS >= 2 && out.watchdogS <= 600)) out.watchdogS = DEFAULT_WATCHDOG_TIMEOUT_S;
    if (!(out.telemetryHz >= 0.5f && out.telemetryHz <= 20)) out.telemetryHz = DEFAULT_TELEMETRY_HZ;
    if (out.pressureUnit > 3) out.pressureUnit = 0;
}

void saveSettings(const AppSettings& s) {
    SettingsBlob b;
    b.version = kSettingsVersion;
    b.s = s;
    prefs.putBytes("settings", &b, sizeof(b));
}

// isKey() first: the Preferences getters log an error for missing keys,
// which is the normal state on first boot.
String loadChannelProfile(int ch) {
    const String k = key("prof", ch);
    return prefs.isKey(k.c_str()) ? prefs.getString(k.c_str(), "") : String();
}
void saveChannelProfile(int ch, const String& id) { prefs.putString(key("prof", ch).c_str(), id); }
float loadZeroOffset(int ch) {
    const String k = key("zoff", ch);
    if (!prefs.isKey(k.c_str())) return 0.0f;
    const float v = prefs.getFloat(k.c_str(), 0.0f);
    return isfinite(v) ? v : 0.0f;
}
void saveZeroOffset(int ch, float v) { prefs.putFloat(key("zoff", ch).c_str(), v); }
float loadPressureZero(int ch) {
    const String k = key("poff", ch);
    if (!prefs.isKey(k.c_str())) return NAN;
    const float v = prefs.getFloat(k.c_str(), NAN);
    return isfinite(v) ? v : NAN;
}
void savePressureZero(int ch, float v) {
    const String k = key("poff", ch);
    if (isfinite(v)) prefs.putFloat(k.c_str(), v);
    else if (prefs.isKey(k.c_str())) prefs.remove(k.c_str());
}
float loadPressureAlarm(int ch) {
    const String k = key("pmax", ch);
    if (!prefs.isKey(k.c_str())) return 0.0f;
    const float v = prefs.getFloat(k.c_str(), 0.0f);
    return (isfinite(v) && v >= 0) ? v : 0.0f;
}
void savePressureAlarm(int ch, float v) { prefs.putFloat(key("pmax", ch).c_str(), v); }

void factoryReset() { prefs.clear(); }

}  // namespace settings_store
