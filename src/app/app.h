#pragma once
// =============================================================================
// app — shared runtime state and the tasks that own it.
//
//   core 1: rtTask      — sensor polling, run state machines, motor ramps
//   core 0: AsyncTCP    — HTTP + WebSocket callbacks (never block)
//           telemetry   — builds/broadcasts WS telemetry
//           storage     — all flash writes (LittleFS profiles, NVS)
//
// One FreeRTOS mutex guards the channels/settings; holders keep it < ~1 ms.
// Flash writes never happen with the mutex held: they're queued as jobs.
// =============================================================================
#include <Arduino.h>
#include "app/channel.h"
#include "config.h"
#include "config/hardware_config.h"
#include "config/settings_store.h"
#include "control/watchdog.h"

namespace app {

// ---------- shared state (guarded by the mutex) ----------
extern Channel channels[MAX_CHANNELS];
extern AppSettings settings;
extern HardwareConfig hardware;
extern HeartbeatWatchdog watchdog;
extern bool estopLatched;

bool lock(uint32_t timeoutMs = 200);
void unlock();
struct Lock {
    bool ok;
    explicit Lock(uint32_t timeoutMs = 200) : ok(lock(timeoutMs)) {}
    ~Lock() {
        if (ok) unlock();
    }
};

// ---------- time ----------
// The ESP32 has no RTC; the browser sends its clock on connect.
void setEpochMs(uint64_t epochMs);
bool timeSynced();
uint64_t epochMs();          // wall clock if synced, else uptime ms
uint32_t epochS();           // 0 if not synced

// ---------- events ----------
struct EventRec {
    uint32_t seq;
    uint64_t tMs;            // epochMs() at the time
    int8_t ch;               // -1 = system
    uint8_t level;           // 0 info, 1 warn, 2 alarm
    char msg[80];
};
void logEvent(int ch, uint8_t level, const char* fmt, ...);
// Copies events with seq > afterSeq (oldest first) into out; returns count.
int eventsSince(uint32_t afterSeq, EventRec* out, int max);
uint32_t lastEventSeq();

// ---------- history (≈30 min at 1 Hz, < 64 KB) ----------
struct HistoryEntry {
    uint32_t tMs;            // uptime ms
    HistoryChannel ch[MAX_CHANNELS];
};
size_t historyCapacity();
size_t historySize();
bool historyAt(size_t i, HistoryEntry& out);   // i = 0 oldest; takes the mutex
void historyClear();
int64_t uptimeToEpochOffsetMs();               // add to uptime to get epoch (0 if unsynced)

// ---------- safety ----------
void estopAll(const char* source);
void resetEstop();

// ---------- hardware reconfiguration (applied by rtTask) ----------
void requestHardware(const HardwareConfig& hw);

// ---------- storage jobs (executed by the storage task, core 0) ----------
enum class JobType : uint8_t {
    SAVE_PROFILE, DELETE_PROFILE, ASSIGN_PROFILE, SAVE_HARDWARE, SAVE_SETTINGS, APPLY_WIFI, SAVE_ZERO,
    SAVE_PRESSURE_ZERO, SAVE_PRESSURE_ALARM,
};
struct Job {
    explicit Job(JobType t) : type(t) {}
    JobType type;
    int ch = -1;
    float value = 0;
    String id;
    PumpProfile* profile = nullptr;   // owned by the job
};
bool enqueue(Job* job);               // takes ownership; false if the queue is full

// ---------- lifecycle ----------
void begin();                         // load settings, create mutex/queues
void startTasks();
const char* firmwareVersion();
bool simMode();

}  // namespace app
