#include <unity.h>
#include "sensors/slf3s_protocol.h"
#include "util/crc8.h"

void setUp() {}
void tearDown() {}

static void putWord(uint8_t* p, uint16_t w) {
    p[0] = w >> 8;
    p[1] = w & 0xFF;
    p[2] = crc8Sensirion(p, 2);
}

static void makeFrame(uint8_t* f, int16_t flow, int16_t temp, uint16_t flags) {
    putWord(f, static_cast<uint16_t>(flow));
    putWord(f + 3, static_cast<uint16_t>(temp));
    putWord(f + 6, flags);
}

void test_parse_scaling() {
    uint8_t f[9];
    makeFrame(f, 500, 4600, 0);   // 1.000 mL/min, 23.00 °C
    slf3s::Sample s;
    TEST_ASSERT_EQUAL(static_cast<int>(slf3s::ParseResult::OK), static_cast<int>(slf3s::parseFrame(f, 9, s)));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1000.0f, s.flowUlMin);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 23.0f, s.tempC);
    TEST_ASSERT_EQUAL(500, s.rawFlow);
}

void test_parse_low_flow_and_negative() {
    uint8_t f[9];
    makeFrame(f, 31, 4000, 0);   // 31/500 mL/min = 62 µL/min
    slf3s::Sample s;
    slf3s::parseFrame(f, 9, s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 62.0f, s.flowUlMin);
    makeFrame(f, -250, -200, 0);   // two's complement
    slf3s::parseFrame(f, 9, s);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -500.0f, s.flowUlMin);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -1.0f, s.tempC);
}

void test_flags_decode() {
    uint8_t f[9];
    makeFrame(f, 0, 4600, slf3s::FLAG_AIR_IN_LINE | slf3s::FLAG_HIGH_FLOW);
    slf3s::Sample s;
    slf3s::parseFrame(f, 9, s);
    TEST_ASSERT_TRUE(s.flags & slf3s::FLAG_AIR_IN_LINE);
    TEST_ASSERT_TRUE(s.flags & slf3s::FLAG_HIGH_FLOW);
    TEST_ASSERT_FALSE(s.flags & slf3s::FLAG_EXP_SMOOTHING);
}

void test_crc_failure_leaves_sample_untouched() {
    uint8_t f[9];
    makeFrame(f, 500, 4600, 0);
    f[4] ^= 0x01;   // corrupt temperature LSB
    slf3s::Sample s;
    s.flowUlMin = -7;
    TEST_ASSERT_EQUAL(static_cast<int>(slf3s::ParseResult::CRC_ERROR), static_cast<int>(slf3s::parseFrame(f, 9, s)));
    TEST_ASSERT_EQUAL_FLOAT(-7, s.flowUlMin);
}

void test_short_frame() {
    uint8_t f[9] = {};
    slf3s::Sample s;
    TEST_ASSERT_EQUAL(static_cast<int>(slf3s::ParseResult::SHORT_FRAME), static_cast<int>(slf3s::parseFrame(f, 6, s)));
}

void test_command_encoding() {
    uint8_t b[2];
    slf3s::encodeCommand(slf3s::CMD_START_CONTINUOUS_WATER, b);
    TEST_ASSERT_EQUAL_HEX8(0x36, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0x08, b[1]);
    slf3s::encodeCommand(slf3s::CMD_STOP_CONTINUOUS, b);
    TEST_ASSERT_EQUAL_HEX8(0x3F, b[0]);
    TEST_ASSERT_EQUAL_HEX8(0xF9, b[1]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_parse_scaling);
    RUN_TEST(test_parse_low_flow_and_negative);
    RUN_TEST(test_flags_decode);
    RUN_TEST(test_crc_failure_leaves_sample_untouched);
    RUN_TEST(test_short_frame);
    RUN_TEST(test_command_encoding);
    return UNITY_END();
}
