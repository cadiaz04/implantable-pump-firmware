#include "app.h"
#include <cstdarg>
#include <WiFi.h>
#include "net/wifi_manager.h"
#include "pumps/profile_store.h"
#include "sensors/sensor_registry.h"

SensorRegistry g_sensors;

namespace app {

Channel channels[MAX_CHANNELS];
AppSettings settings;
HardwareConfig hardware;
HeartbeatWatchdog watchdog;
bool estopLatched = false;

namespace {
SemaphoreHandle_t g_mtx;
QueueHandle_t g_jobs;
portMUX_TYPE g_evMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool g_hwPending = false;
HardwareConfig g_hwNext;

int64_t g_epochOffsetMs = 0;
bool g_timeSynced = false;

constexpr int kEvents = 48;
EventRec g_events[kEvents];
int g_evHead = 0, g_evCount = 0;
uint32_t g_evSeq = 0;

HistoryEntry* g_hist = nullptr;
size_t g_histCap = 0, g_histHead = 0, g_histSize = 0;
}  // namespace

const char* firmwareVersion() { return "1.0.0"; }
bool simMode() {
#ifdef SIM_MODE
    return true;
#else
    return false;
#endif
}

bool lock(uint32_t timeoutMs) { return xSemaphoreTake(g_mtx, pdMS_TO_TICKS(timeoutMs)) == pdTRUE; }
void unlock() { xSemaphoreGive(g_mtx); }

void setEpochMs(uint64_t e) {
    g_epochOffsetMs = static_cast<int64_t>(e) - static_cast<int64_t>(millis());
    g_timeSynced = true;
}
bool timeSynced() { return g_timeSynced; }
uint64_t epochMs() { return static_cast<uint64_t>(static_cast<int64_t>(millis()) + (g_timeSynced ? g_epochOffsetMs : 0)); }
uint32_t epochS() { return g_timeSynced ? static_cast<uint32_t>(epochMs() / 1000) : 0; }
int64_t uptimeToEpochOffsetMs() { return g_timeSynced ? g_epochOffsetMs : 0; }

void logEvent(int ch, uint8_t level, const char* fmt, ...) {
    EventRec e;
    e.ch = static_cast<int8_t>(ch);
    e.level = level;
    e.tMs = epochMs();
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e.msg, sizeof(e.msg), fmt, ap);
    va_end(ap);
    portENTER_CRITICAL(&g_evMux);
    e.seq = ++g_evSeq;
    g_events[(g_evHead + g_evCount) % kEvents] = e;
    if (g_evCount == kEvents) g_evHead = (g_evHead + 1) % kEvents;
    else ++g_evCount;
    portEXIT_CRITICAL(&g_evMux);
    Serial.printf("[event] %s%s %s\n", ch >= 0 ? "Ch" : "", ch >= 0 ? String(ch + 1).c_str() : "sys", e.msg);
}

int eventsSince(uint32_t afterSeq, EventRec* out, int max) {
    int n = 0;
    portENTER_CRITICAL(&g_evMux);
    for (int i = 0; i < g_evCount && n < max; ++i) {
        const EventRec& e = g_events[(g_evHead + i) % kEvents];
        if (e.seq > afterSeq) out[n++] = e;
    }
    portEXIT_CRITICAL(&g_evMux);
    return n;
}

uint32_t lastEventSeq() { return g_evSeq; }

size_t historyCapacity() { return g_histCap; }
size_t historySize() { return g_histSize; }

bool historyAt(size_t i, HistoryEntry& out) {
    Lock l(50);
    if (!l.ok || i >= g_histSize) return false;
    out = g_hist[(g_histHead + g_histCap - g_histSize + i) % g_histCap];
    return true;
}

void historyClear() {
    Lock l;
    g_histSize = 0;
    g_histHead = 0;
}

static void historyPush(const HistoryEntry& e) {
    if (!g_hist) return;
    g_hist[g_histHead] = e;
    g_histHead = (g_histHead + 1) % g_histCap;
    if (g_histSize < g_histCap) ++g_histSize;
}

void estopAll(const char* source) {
    // Callers must NOT hold the mutex. If the lock can't be had quickly we
    // stop the motors anyway — stopping wins over consistency.
    Lock l(100);
    const bool already = estopLatched;
    estopLatched = true;
    for (auto& c : channels) c.estop();
    // The UI sends E-STOP over both WebSocket and REST; log it once.
    if (!already) logEvent(-1, 2, "E-STOP (%s): all motors stopped, outputs disabled", source);
}

void resetEstop() {
    Lock l;
    if (!estopLatched) return;
    estopLatched = false;
    for (auto& c : channels) c.resetEstop();
    logEvent(-1, 1, "E-STOP reset: channels IDLE (nothing restarts automatically)");
}

void requestHardware(const HardwareConfig& hw) {
    g_hwNext = hw;
    g_hwPending = true;
}

bool enqueue(Job* job) {
    if (xQueueSend(g_jobs, &job, 0) != pdTRUE) {
        delete job->profile;
        delete job;
        return false;
    }
    return true;
}

// ---------------- real-time task (core 1) ----------------
static void rtTask(void*) {
    // Build the drivers here so step-engine interrupts are serviced by core 1.
    {
        Lock l(portMAX_DELAY);
        for (int i = 0; i < MAX_CHANNELS; ++i) channels[i].applyHardware(hardware.ch[i]);
    }
    g_sensors.beginAll();
    TickType_t wake = xTaskGetTickCount();
    uint32_t last = millis();
    uint32_t lastHist = last;
    for (;;) {
        const uint32_t now = millis();
        const float dt = (now - last) / 1000.0f;
        last = now;

        for (auto& c : channels) c.pollSensor(now);   // I²C, outside the mutex
        g_sensors.pollAll(now);

        if (lock(50)) {
            if (g_hwPending) {
                g_hwPending = false;
                hardware = g_hwNext;
                for (int i = 0; i < MAX_CHANNELS; ++i) channels[i].applyHardware(hardware.ch[i]);
                if (estopLatched)
                    for (auto& c : channels) c.estop();
                logEvent(-1, 0, "Hardware configuration applied");
            }
            const bool hbExpired = watchdog.expired(now, settings.watchdogS);
            for (int i = 0; i < MAX_CHANNELS; ++i) {
                Channel& c = channels[i];
                c.tick(now, dt, hbExpired);
                RunEvent ev;
                while (c.popEvent(ev)) logEvent(i, ev.level, "%s", ev.msg);
                float z;
                if (c.takeZeroSave(z)) {
                    Job* j = new Job(JobType::SAVE_ZERO);
                    j->ch = i;
                    j->value = z;
                    enqueue(j);
                }
                if (c.takePressureZeroSave(z)) {
                    Job* j = new Job(JobType::SAVE_PRESSURE_ZERO);
                    j->ch = i;
                    j->value = z;
                    enqueue(j);
                }
            }
            if (now - lastHist >= HISTORY_PERIOD_MS) {
                lastHist += HISTORY_PERIOD_MS;
                if (now - lastHist > 5 * HISTORY_PERIOD_MS) lastHist = now;
                HistoryEntry e;
                e.tMs = now;
                for (int i = 0; i < MAX_CHANNELS; ++i) channels[i].takeHistory(e.ch[i]);
                historyPush(e);
            }
            bool anyActive = false;
            for (auto& c : channels) anyActive |= !c.isIdle();
            unlock();
            // Status LED: solid = E-STOP, fast blink = running, slow = idle.
            const uint32_t period = estopLatched ? 0 : (anyActive ? 250 : 1000);
            digitalWrite(PIN_STATUS_LED, period == 0 ? HIGH : ((now / period) & 1));
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

// ---------------- storage task (core 0) ----------------
static void applyProfileToChannels(const PumpProfile& p) {
    Lock l(1000);
    for (auto& c : channels) {
        if (c.hasProfile() && c.profile().id == p.id) c.setProfile(p);
    }
}

static void storageTask(void*) {
    for (;;) {
        Job* j = nullptr;
        if (xQueueReceive(g_jobs, &j, portMAX_DELAY) != pdTRUE || !j) continue;
        String err;
        switch (j->type) {
            case JobType::SAVE_PROFILE:
                if (j->profile) {
                    if (g_profiles.save(*j->profile, &err)) applyProfileToChannels(*j->profile);
                    else logEvent(-1, 2, "Saving profile %s failed: %s", j->profile->id.c_str(), err.c_str());
                }
                break;
            case JobType::DELETE_PROFILE:
                g_profiles.remove(j->id);
                logEvent(-1, 0, "Profile %s deleted", j->id.c_str());
                break;
            case JobType::ASSIGN_PROFILE: {
                PumpProfile p;
                if (g_profiles.load(j->id, p, &err)) {
                    {
                        Lock l(1000);
                        channels[j->ch].setProfile(p);
                    }
                    settings_store::saveChannelProfile(j->ch, j->id);
                    logEvent(j->ch, 0, "Pump profile: %s", p.name.c_str());
                } else {
                    logEvent(j->ch, 2, "Cannot load profile %s: %s", j->id.c_str(), err.c_str());
                }
                break;
            }
            case JobType::SAVE_HARDWARE: {
                HardwareConfig hw;
                {
                    Lock l(1000);
                    hw = hardware;
                }
                settings_store::saveHardware(hw);
                break;
            }
            case JobType::SAVE_SETTINGS: {
                AppSettings s;
                {
                    Lock l(1000);
                    s = settings;
                }
                settings_store::saveSettings(s);
                break;
            }
            case JobType::APPLY_WIFI: {
                AppSettings s;
                {
                    Lock l(1000);
                    s = settings;
                }
                wifi_manager::applyStation(s);
                break;
            }
            case JobType::SAVE_ZERO:
                settings_store::saveZeroOffset(j->ch, j->value);
                break;
            case JobType::SAVE_PRESSURE_ZERO:
                settings_store::savePressureZero(j->ch, j->value);
                break;
            case JobType::SAVE_PRESSURE_ALARM:
                settings_store::savePressureAlarm(j->ch, j->value);
                break;
        }
        delete j->profile;
        delete j;
    }
}

void begin() {
    g_mtx = xSemaphoreCreateMutex();
    g_jobs = xQueueCreate(16, sizeof(Job*));
    // History ring: sized from the RAM budget; allocate early, before the
    // heap fragments.
    g_histCap = HISTORY_RAM_BUDGET_BYTES / sizeof(HistoryEntry);
    g_hist = static_cast<HistoryEntry*>(malloc(g_histCap * sizeof(HistoryEntry)));
    if (!g_hist) {
        g_histCap = 600;
        g_hist = static_cast<HistoryEntry*>(malloc(g_histCap * sizeof(HistoryEntry)));
        if (!g_hist) g_histCap = 0;
    }
    settings_store::begin();
    settings_store::loadSettings(settings);
    if (!settings_store::loadHardware(hardware)) Serial.println("[cfg] using default hardware config");
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        channels[i].init(i);
        channels[i].setZeroOffset(settings_store::loadZeroOffset(i));
        channels[i].setPressureZero(settings_store::loadPressureZero(i));
        channels[i].setPressureAlarm(settings_store::loadPressureAlarm(i));
        channels[i].setSensorFaultStops(settings.sensorFaultStops);
    }
}

void startTasks() {
    // Profiles: load the saved assignment (or a sensible default per driver).
    for (int i = 0; i < MAX_CHANNELS; ++i) {
        String id = settings_store::loadChannelProfile(i);
        PumpProfile p;
        bool ok = !id.isEmpty() && g_profiles.load(id, p);
        if (ok && familyOf(p.motor.driverType) != familyOf(hardware.ch[i].driver)) ok = false;
        if (!ok) {
            // Default: a profile for the exact driver type, else the same family.
            const auto all = g_profiles.list();
            for (int pass = 0; pass < 2 && !ok; ++pass) {
                for (const auto& s : all) {
                    const bool match = pass == 0 ? s.driver == hardware.ch[i].driver
                                                 : familyOf(s.driver) == familyOf(hardware.ch[i].driver);
                    if (match && g_profiles.load(s.id, p)) {
                        ok = true;
                        break;
                    }
                }
            }
        }
        if (ok) channels[i].setProfile(p);
    }
    xTaskCreatePinnedToCore(rtTask, "rt", 6144, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(storageTask, "storage", 8192, nullptr, 1, nullptr, 0);
}

}  // namespace app
