#include "step_engine.h"
#include <Arduino.h>
#include <driver/timer.h>
#include <soc/gpio_struct.h>
#include "config.h"

namespace step_engine {
namespace {

constexpr uint32_t kTimerHz = 10000000;     // APB 80 MHz / 8
constexpr uint32_t kMaxChunk = 10000;       // 1 ms: re-check speed at least this often
constexpr uint32_t kMinEvent = 200;         // 20 µs → 25 kHz toggle cap

struct Slot {
    volatile uint32_t eventTicks;   // ticks between events, 0 = stopped
    volatile bool reverse;
    uint32_t remaining;
    uint32_t lastChunk;
    Mode mode;
    bool attached;
    // STEP_DIR
    uint32_t stepLo, stepHi;
    bool stepHigh;
    int dirPin;
    // ULN4: per-phase set/clear masks for GPIO 0-31 (lo) and 32-39 (hi)
    uint32_t setLo[8], clrLo[8], setHi[8], clrHi[8];
    uint32_t allLo, allHi;
    uint8_t phase;
    volatile int32_t position;
};

DRAM_ATTR Slot g_slots[kSlots];
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Half-step sequence, bit i = IN(i+1)
constexpr uint8_t kHalfStep[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};

inline void IRAM_ATTR writeMasks(uint32_t setLo, uint32_t clrLo, uint32_t setHi, uint32_t clrHi) {
    if (clrLo) GPIO.out_w1tc = clrLo;
    if (clrHi) GPIO.out1_w1tc.val = clrHi;
    if (setLo) GPIO.out_w1ts = setLo;
    if (setHi) GPIO.out1_w1ts.val = setHi;
}

void IRAM_ATTR doEvent(Slot* s) {
    if (s->mode == Mode::STEP_DIR) {
        if (s->stepHigh) {
            writeMasks(0, s->stepLo, 0, s->stepHi);
            s->stepHigh = false;
        } else {
            writeMasks(s->stepLo, 0, s->stepHi, 0);
            s->stepHigh = true;
            s->position += s->reverse ? -1 : 1;
        }
    } else {
        s->phase = (s->phase + (s->reverse ? 7 : 1)) & 7;
        const uint8_t p = s->phase;
        writeMasks(s->setLo[p], s->clrLo[p], s->setHi[p], s->clrHi[p]);
        s->position += s->reverse ? -1 : 1;
    }
}

bool IRAM_ATTR onTimer(void* arg) {
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(arg));
    Slot* s = &g_slots[idx];
    portENTER_CRITICAL_ISR(&g_mux);
    const uint32_t ev = s->eventTicks;
    s->remaining = s->remaining > s->lastChunk ? s->remaining - s->lastChunk : 0;
    uint32_t chunk;
    if (ev == 0) {
        if (s->mode == Mode::STEP_DIR && s->stepHigh) {
            writeMasks(0, s->stepLo, 0, s->stepHi);
            s->stepHigh = false;
        }
        s->remaining = 0;
        chunk = kMaxChunk;
    } else {
        if (s->remaining > ev) s->remaining = ev;   // speed-ups apply immediately
        if (s->remaining == 0) {
            doEvent(s);
            s->remaining = ev;
        }
        chunk = s->remaining < kMaxChunk ? s->remaining : kMaxChunk;
    }
    s->lastChunk = chunk;
    portEXIT_CRITICAL_ISR(&g_mux);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, static_cast<timer_idx_t>(idx), chunk);
    return false;
}

void maskFor(int pin, uint32_t& lo, uint32_t& hi) {
    lo = hi = 0;
    if (pin < 0) return;
    if (pin < 32) lo = 1u << pin;
    else hi = 1u << (pin - 32);
}

}  // namespace

bool attach(int slot, Mode mode, const int* pins) {
    if (slot < 0 || slot >= kSlots) return false;
    detach(slot);
    Slot& s = g_slots[slot];
    const int n = mode == Mode::ULN4 ? 4 : 2;
    for (int i = 0; i < n; ++i) {
        if (pins[i] < 0) return false;
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
    portENTER_CRITICAL(&g_mux);
    s = Slot{};
    s.mode = mode;
    if (mode == Mode::STEP_DIR) {
        maskFor(pins[0], s.stepLo, s.stepHi);
        s.dirPin = pins[1];
    } else {
        uint32_t lo[4], hi[4];
        for (int i = 0; i < 4; ++i) {
            maskFor(pins[i], lo[i], hi[i]);
            s.allLo |= lo[i];
            s.allHi |= hi[i];
        }
        for (int p = 0; p < 8; ++p) {
            for (int i = 0; i < 4; ++i) {
                const bool on = kHalfStep[p] & (1u << i);
                (on ? s.setLo[p] : s.clrLo[p]) |= lo[i];
                (on ? s.setHi[p] : s.clrHi[p]) |= hi[i];
            }
        }
        s.phase = 0;
    }
    s.attached = true;
    portEXIT_CRITICAL(&g_mux);

    timer_config_t cfg = {};
    cfg.divider = 80000000 / kTimerHz;
    cfg.counter_dir = TIMER_COUNT_UP;
    cfg.counter_en = TIMER_PAUSE;
    cfg.alarm_en = TIMER_ALARM_EN;
    cfg.auto_reload = TIMER_AUTORELOAD_EN;
    cfg.intr_type = TIMER_INTR_LEVEL;
    const timer_idx_t t = static_cast<timer_idx_t>(slot);
    if (timer_init(TIMER_GROUP_0, t, &cfg) != ESP_OK) return false;
    timer_set_counter_value(TIMER_GROUP_0, t, 0);
    timer_set_alarm_value(TIMER_GROUP_0, t, kMaxChunk);
    g_slots[slot].lastChunk = kMaxChunk;
    timer_enable_intr(TIMER_GROUP_0, t);
    timer_isr_callback_add(TIMER_GROUP_0, t, onTimer, reinterpret_cast<void*>(static_cast<intptr_t>(slot)), ESP_INTR_FLAG_IRAM);
    timer_start(TIMER_GROUP_0, t);
    return true;
}

void detach(int slot) {
    if (slot < 0 || slot >= kSlots) return;
    Slot& s = g_slots[slot];
    if (!s.attached) return;
    const timer_idx_t t = static_cast<timer_idx_t>(slot);
    timer_pause(TIMER_GROUP_0, t);
    timer_disable_intr(TIMER_GROUP_0, t);
    timer_isr_callback_remove(TIMER_GROUP_0, t);
    timer_deinit(TIMER_GROUP_0, t);
    portENTER_CRITICAL(&g_mux);
    s.eventTicks = 0;
    if (s.mode == Mode::STEP_DIR) writeMasks(0, s.stepLo, 0, s.stepHi);
    else writeMasks(0, s.allLo, 0, s.allHi);
    s.attached = false;
    portEXIT_CRITICAL(&g_mux);
}

void setRate(int slot, float sps, bool reverse) {
    if (slot < 0 || slot >= kSlots) return;
    Slot& s = g_slots[slot];
    if (!s.attached) return;
    if (!(sps > 0)) sps = 0;
    if (sps > ABS_MAX_STEP_RATE_HZ) sps = ABS_MAX_STEP_RATE_HZ;
    uint32_t ev = 0;
    if (sps > 0) {
        // STEP_DIR toggles twice per step.
        const float eventsPerSec = s.mode == Mode::STEP_DIR ? 2.0f * sps : sps;
        const float ticks = kTimerHz / eventsPerSec;
        ev = ticks > 4.0e9f ? 4000000000u : static_cast<uint32_t>(ticks);
        if (ev < kMinEvent) ev = kMinEvent;
    }
    if (s.mode == Mode::STEP_DIR && s.dirPin >= 0 && reverse != s.reverse) digitalWrite(s.dirPin, reverse ? HIGH : LOW);
    portENTER_CRITICAL(&g_mux);
    s.reverse = reverse;
    s.eventTicks = ev;
    portEXIT_CRITICAL(&g_mux);
}

void release(int slot) {
    if (slot < 0 || slot >= kSlots) return;
    Slot& s = g_slots[slot];
    if (!s.attached || s.mode != Mode::ULN4) return;
    portENTER_CRITICAL(&g_mux);
    if (s.eventTicks == 0) writeMasks(0, s.allLo, 0, s.allHi);
    portEXIT_CRITICAL(&g_mux);
}

int32_t position(int slot) {
    if (slot < 0 || slot >= kSlots) return 0;
    return g_slots[slot].position;
}

}  // namespace step_engine
