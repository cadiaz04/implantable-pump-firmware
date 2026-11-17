#include "mprls_protocol.h"

namespace mprls {

float countsToPressure(uint32_t counts, uint32_t outMin, uint32_t outMax, float pMin, float pMax) {
    const double num = (static_cast<double>(counts) - outMin) * (pMax - pMin);
    return static_cast<float>(num / (static_cast<double>(outMax) - outMin) + pMin);
}

Result parseFrame(const uint8_t* buf, size_t len, Sample& out) {
    if (len < FRAME_BYTES) return Result::BAD_FRAME;
    const uint8_t st = buf[0];
    out.status = st;
    // The MPR has no CRC; the always-zero and power bits are the only
    // sanity check against a garbled frame.
    if ((st & STATUS_ALWAYS_ZERO) || !(st & STATUS_POWERED)) return Result::BAD_FRAME;
    if (st & STATUS_BUSY) return Result::BUSY;
    if (st & STATUS_INTEGRITY_FAIL) return Result::INTEGRITY_FAIL;
    out.raw = (static_cast<uint32_t>(buf[1]) << 16) | (static_cast<uint32_t>(buf[2]) << 8) | buf[3];
    out.psiAbs = countsToPressure(out.raw, OUTPUT_MIN, OUTPUT_MAX, P_MIN_PSI, P_MAX_PSI);
    out.kPaAbs = out.psiAbs * KPA_PER_PSI;
    if (st & STATUS_MATH_SATURATION) return Result::SATURATED;
    return Result::OK;
}

}  // namespace mprls
