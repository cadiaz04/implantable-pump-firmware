#include "slf3s_protocol.h"
#include "util/crc8.h"

namespace slf3s {

ParseResult parseFrame(const uint8_t* buf, size_t len, Sample& out) {
    if (len < FRAME_BYTES) return ParseResult::SHORT_FRAME;
    for (int w = 0; w < 3; ++w) {
        if (crc8Sensirion(buf + 3 * w, 2) != buf[3 * w + 2]) return ParseResult::CRC_ERROR;
    }
    const int16_t rawFlow = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    const int16_t rawTemp = static_cast<int16_t>((buf[3] << 8) | buf[4]);
    const uint16_t flags = static_cast<uint16_t>((buf[6] << 8) | buf[7]);
    out.rawFlow = rawFlow;
    out.rawTemp = rawTemp;
    out.flags = flags;
    out.flowUlMin = rawFlow / FLOW_SCALE * 1000.0f;
    out.tempC = rawTemp / TEMP_SCALE;
    return ParseResult::OK;
}

}  // namespace slf3s
