#pragma once
// =============================================================================
// mprls_protocol.h — Honeywell MPR series (MPRLS0025PA00001A, as on the
// SparkFun Qwiic MicroPressure board) constants and frame decoding.
// All constants verified against the Honeywell MPR Series datasheet
// (see docs/ASSUMPTIONS.md). Pure logic, unit tested natively.
// =============================================================================
#include <cstddef>
#include <cstdint>

namespace mprls {

constexpr uint8_t I2C_ADDRESS = 0x18;               // "…0001A" = I²C, address 0x18
constexpr uint8_t CMD_MEASURE[3] = {0xAA, 0x00, 0x00};
constexpr uint32_t CONVERSION_MS = 5;               // datasheet: wait ≥ 5 ms or poll busy
constexpr uint32_t POWER_UP_MS = 3;                 // first measurement 2.5 ms after VDD

// Transfer function A: 10 % to 90 % of 2^24 counts.
constexpr uint32_t OUTPUT_MIN = 1677722;            // 0x19999A
constexpr uint32_t OUTPUT_MAX = 15099494;           // 0xE66666
constexpr float P_MIN_PSI = 0.0f;                   // 0025PA: 0-25 psi absolute
constexpr float P_MAX_PSI = 25.0f;

constexpr float KPA_PER_PSI = 6.894757f;
constexpr float STD_ATMOSPHERE_KPA = 101.325f;

// Status byte (datasheet Table 9)
constexpr uint8_t STATUS_POWERED = 1u << 6;         // 1 = device is powered
constexpr uint8_t STATUS_BUSY = 1u << 5;            // 1 = conversion not finished
constexpr uint8_t STATUS_INTEGRITY_FAIL = 1u << 2;  // 1 = memory checksum failed (set at power-up)
constexpr uint8_t STATUS_MATH_SATURATION = 1u << 0; // 1 = internal math saturated (out of range)
constexpr uint8_t STATUS_ALWAYS_ZERO = (1u << 7) | (1u << 4) | (1u << 3) | (1u << 1);

constexpr size_t FRAME_BYTES = 4;                   // status + 24-bit pressure

enum class Result : uint8_t {
    OK,
    BUSY,               // try again later — not an error
    SATURATED,          // pressure outside the sensor's range (value unusable)
    INTEGRITY_FAIL,     // sensor memory corrupt → FAULT
    BAD_FRAME,          // not powered / always-zero bits set / short read (the MPR has no CRC)
};

struct Sample {
    uint8_t status = 0;
    uint32_t raw = 0;
    float psiAbs = 0;
    float kPaAbs = 0;
};

// Equation 2 of the datasheet, generic over range and output limits.
float countsToPressure(uint32_t counts, uint32_t outMin, uint32_t outMax, float pMin, float pMax);

Result parseFrame(const uint8_t* buf, size_t len, Sample& out);

}  // namespace mprls
