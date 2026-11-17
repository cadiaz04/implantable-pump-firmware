#pragma once
// =============================================================================
// step_engine — jitter-free step generation from hardware-timer interrupts.
//
// Each stepper slot owns one ESP32 timer-group timer (10 MHz tick). The ISR
// (in IRAM, so it keeps running during flash writes) fires exactly when the
// next step event is due, so step timing jitter is only the interrupt latency
// (~1 µs), comfortably supporting > 10 kHz STEP/DIR rates. Speed changes are
// picked up within 1 ms even at very low step rates.
//
//   STEP_DIR : event = toggle STEP (2 events per step, 50 % duty)
//   ULN4     : event = advance the 8-phase half-step sequence on IN1..IN4
//
// attach() must be called from the core that should service the interrupt
// (the real-time task, core 1).
// =============================================================================
#include <cstdint>

namespace step_engine {

constexpr int kSlots = 2;
enum class Mode : uint8_t { STEP_DIR, ULN4 };

// pins: STEP_DIR → {step, dir}; ULN4 → {in1, in2, in3, in4}
bool attach(int slot, Mode mode, const int* pins);
void detach(int slot);
// Steps per second (≥ 0). 0 stops. Direction applies immediately; callers
// ramp through zero before reversing.
void setRate(int slot, float stepsPerSec, bool reverse);
// ULN4: drive all coils low (no holding current). STEP_DIR: no-op.
void release(int slot);
int32_t position(int slot);

}  // namespace step_engine
