#include <unity.h>
#include <cmath>
#include "util/crc8.h"
#include "util/ring_buffer.h"
#include "util/totalizer.h"
#include "util/units.h"
#include "util/window_stats.h"

void setUp() {}
void tearDown() {}

// Datasheet example: CRC of 0xBEEF is 0x92.
void test_crc8_datasheet_vector() {
    const uint8_t d[] = {0xBE, 0xEF};
    TEST_ASSERT_EQUAL_HEX8(0x92, crc8Sensirion(d, 2));
}

void test_crc8_more_vectors() {
    const uint8_t zero[] = {0x00, 0x00};
    TEST_ASSERT_EQUAL_HEX8(0x81, crc8Sensirion(zero, 2));
    const uint8_t ff[] = {0xFF, 0xFF};
    TEST_ASSERT_EQUAL_HEX8(0xAC, crc8Sensirion(ff, 2));
    TEST_ASSERT_EQUAL_HEX8(0xFF, crc8Sensirion(nullptr, 0));
}

void test_ring_buffer_wraps() {
    RingBuffer<int, 4> rb;
    for (int i = 1; i <= 6; ++i) rb.push(i);
    TEST_ASSERT_EQUAL(4, rb.size());
    TEST_ASSERT_EQUAL(3, rb.at(0));
    TEST_ASSERT_EQUAL(6, rb.at(3));
    TEST_ASSERT_EQUAL(6, rb.newest());
}

void test_totalizer_constant_flow() {
    Totalizer t;
    // 60 µL/min for 120 s at 50 Hz → 120 µL
    for (int i = 0; i <= 6000; ++i) t.add(i * 0.02f, 60.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 120.0, t.volumeUl());
}

void test_totalizer_ramp_trapezoid_and_gap() {
    Totalizer t;
    t.add(0, 0);
    t.add(60, 120);   // gap > 1 s → not bridged
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0, t.volumeUl());
    t.reset();
    for (int i = 0; i <= 60; ++i) t.add(i, 2.0f * i);   // linear 0→120 µL/min over 60 s
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 60.0, t.volumeUl());   // ∫ = ½·120·60/60
}

void test_units() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 62.5f, mlHrToUlMin(3.75f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3.75f, ulMinToMlHr(62.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0625f, ulMinToMlMin(62.5f));
}

void test_window_stats_mean_std_slope() {
    WindowStats w(10.0f);
    // alternating 90/110 → mean 100, std ≈ 10
    for (int i = 0; i < 500; ++i) w.add(i * 0.02f, (i % 2) ? 110.0f : 90.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5, 100.0, w.mean());
    TEST_ASSERT_FLOAT_WITHIN(0.2, 10.0, w.stddev());
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, w.slopePerS());

    WindowStats r(20.0f);
    for (int i = 0; i < 1000; ++i) r.add(i * 0.02f, 5.0f + 2.0f * i * 0.02f);   // slope 2/s
    TEST_ASSERT_FLOAT_WITHIN(0.05, 2.0, r.slopePerS());
}

void test_window_stats_forgets_old_data() {
    WindowStats w(5.0f);
    for (int i = 0; i < 500; ++i) w.add(i * 0.02f, 0.0f);          // 0..10 s at 0
    for (int i = 500; i < 1000; ++i) w.add(i * 0.02f, 50.0f);      // 10..20 s at 50
    TEST_ASSERT_FLOAT_WITHIN(1.0, 50.0, w.mean());
    TEST_ASSERT_TRUE(w.spanS() <= 5.5f);
}

void test_ema() {
    Ema e(1.0f);
    e.update(0, 0.01f);
    for (int i = 0; i < 1000; ++i) e.update(10, 0.01f);   // 10 τ
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, e.value());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_datasheet_vector);
    RUN_TEST(test_crc8_more_vectors);
    RUN_TEST(test_ring_buffer_wraps);
    RUN_TEST(test_totalizer_constant_flow);
    RUN_TEST(test_totalizer_ramp_trapezoid_and_gap);
    RUN_TEST(test_units);
    RUN_TEST(test_window_stats_mean_std_slope);
    RUN_TEST(test_window_stats_forgets_old_data);
    RUN_TEST(test_ema);
    return UNITY_END();
}
