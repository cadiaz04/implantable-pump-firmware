#pragma once
// =============================================================================
// slf3s_protocol.h — Sensirion SLF3S-1300F constants and frame decoding.
// ALL sensor constants live in this one header (see docs/ASSUMPTIONS.md for
// how each was verified against the datasheet).
// Pure logic, unit tested natively.
// =============================================================================
#include <cstddef>
#include <cstdint>

namespace slf3s {

constexpr uint8_t I2C_ADDRESS = 0x08;              // fixed — every SLF3S uses 0x08

// 16-bit commands, sent big-endian.
constexpr uint16_t CMD_START_CONTINUOUS_WATER = 0x3608;
constexpr uint16_t CMD_START_CONTINUOUS_IPA = 0x3615;
constexpr uint16_t CMD_STOP_CONTINUOUS = 0x3FF9;
constexpr uint16_t CMD_READ_PRODUCT_ID_1 = 0x367C;
constexpr uint16_t CMD_READ_PRODUCT_ID_2 = 0xE102;
constexpr uint8_t GENERAL_CALL_ADDRESS = 0x00;     // soft reset = 0x06 to general call
constexpr uint8_t GENERAL_CALL_RESET = 0x06;

// Timing
constexpr uint32_t POWER_UP_MS = 25;               // after power-up / soft reset
constexpr uint32_t FIRST_SAMPLE_MS = 60;           // after start command
constexpr uint32_t MAX_READ_INTERVAL_MS = 100;     // read at least this often

// SLF3S-1300F scale factors
constexpr float FLOW_SCALE = 500.0f;               // raw / 500 = mL/min
constexpr float TEMP_SCALE = 200.0f;               // raw / 200 = °C
constexpr float FULL_SCALE_ML_MIN = 40.0f;         // ±40 mL/min

// Signaling flags (third word)
constexpr uint16_t FLAG_AIR_IN_LINE = 1u << 0;
constexpr uint16_t FLAG_HIGH_FLOW = 1u << 1;
constexpr uint16_t FLAG_EXP_SMOOTHING = 1u << 5;

constexpr size_t FRAME_BYTES = 9;                  // 3 × (2 data + 1 CRC)

struct Sample {
    int16_t rawFlow = 0;
    int16_t rawTemp = 0;
    uint16_t flags = 0;
    float flowUlMin = 0;    // µL/min
    float tempC = 0;
};

enum class ParseResult : uint8_t { OK, CRC_ERROR, SHORT_FRAME };

// Decodes a 9-byte continuous-measurement frame. On CRC failure `out` is
// left untouched.
ParseResult parseFrame(const uint8_t* buf, size_t len, Sample& out);

// Writes the 2 command bytes (big-endian) into out[0..1].
inline void encodeCommand(uint16_t cmd, uint8_t* out) {
    out[0] = static_cast<uint8_t>(cmd >> 8);
    out[1] = static_cast<uint8_t>(cmd & 0xFF);
}

}  // namespace slf3s
