// MPRLS protocol decoding, simulated line pressure, and the RunManager's
// over-pressure (occlusion) protection.
#include <unity.h>
#include <cmath>
#include "../support/harness.h"
#include "sensors/mprls_protocol.h"

void setUp() {}
void tearDown() {}

static void frame(uint8_t* b, uint8_t status, uint32_t raw) {
    b[0] = status;
    b[1] = (raw >> 16) & 0xFF;
    b[2] = (raw >> 8) & 0xFF;
    b[3] = raw & 0xFF;
}

// Worked example from the Honeywell MPR datasheet, section 4.0:
// −1..+1 psi gauge, 10-90 % transfer function, 14260634 counts → 0.875 psi.
void test_transfer_function_datasheet_example() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.875f, mprls::countsToPressure(14260634, mprls::OUTPUT_MIN, mprls::OUTPUT_MAX, -1.0f, 1.0f));
}

void test_transfer_function_0025pa_endpoints() {
    uint8_t b[4];
    mprls::Sample s;
    frame(b, mprls::STATUS_POWERED, mprls::OUTPUT_MIN);
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::OK), static_cast<int>(mprls::parseFrame(b, 4, s)));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, s.psiAbs);
    frame(b, mprls::STATUS_POWERED, mprls::OUTPUT_MAX);
    mprls::parseFrame(b, 4, s);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 25.0f, s.psiAbs);
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 172.369f, s.kPaAbs);
    // One standard atmosphere (14.696 psi) ≈ 9 568 255 counts
    frame(b, mprls::STATUS_POWERED, 9568255);
    mprls::parseFrame(b, 4, s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 101.325f, s.kPaAbs);
}

void test_status_decoding() {
    uint8_t b[4];
    mprls::Sample s;
    frame(b, mprls::STATUS_POWERED | mprls::STATUS_BUSY, 0);
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::BUSY), static_cast<int>(mprls::parseFrame(b, 4, s)));
    frame(b, mprls::STATUS_POWERED | mprls::STATUS_INTEGRITY_FAIL, 9568255);
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::INTEGRITY_FAIL), static_cast<int>(mprls::parseFrame(b, 4, s)));
    frame(b, mprls::STATUS_POWERED | mprls::STATUS_MATH_SATURATION, 0xFFFFFF);
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::SATURATED), static_cast<int>(mprls::parseFrame(b, 4, s)));
}

void test_garbled_frames_rejected() {
    uint8_t b[4];
    mprls::Sample s;
    frame(b, 0x00, 9568255);   // power bit missing (e.g. bus reads all zeros)
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::BAD_FRAME), static_cast<int>(mprls::parseFrame(b, 4, s)));
    frame(b, 0xFF, 0xFFFFFF);  // always-zero bits set (e.g. bus reads all ones)
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::BAD_FRAME), static_cast<int>(mprls::parseFrame(b, 4, s)));
    TEST_ASSERT_EQUAL(static_cast<int>(mprls::Result::BAD_FRAME), static_cast<int>(mprls::parseFrame(b, 3, s)));
}

void test_sim_pressure_tracks_flow_and_occlusion() {
    SimPlant p;
    SimPlantParams q;
    q.gain = 20;
    q.noiseUlMin = 0;
    q.pressureNoiseKpa = 0;
    p.setParams(q);
    p.setCommand(5, false);   // 100 µL/min
    for (int i = 0; i < 1500; ++i) p.step(0.02f);   // 30 s
    const float base = p.pressureAbsKpa() - q.atmosphereKpa;
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 100 * q.resistanceKpaPerUlMin, base);
    p.setOcclusion(true);
    for (int i = 0; i < 1500; ++i) p.step(0.02f);
    TEST_ASSERT_TRUE(p.pressureAbsKpa() - q.atmosphereKpa > 30 * base);
    TEST_ASSERT_TRUE(p.measured() < 20);   // almost nothing gets through
}

static RunRequest manual(float c) {
    RunRequest r;
    r.mode = RunMode::MANUAL;
    r.command = c;
    r.keepRunning = true;
    return r;
}

static void setLimit(Harness& h, float kPa, bool stops = false) {
    RunConfig c = h.rm.config();
    c.pressureAlarmKpa = kPa;
    c.sensorFaultStops = stops;
    h.rm.configure(c);
}

void test_no_limit_no_action() {
    Harness h;
    h.pressurePresent = true;
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.plant.setOcclusion(true);
    h.run(30);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
    TEST_ASSERT_FALSE(h.rm.alarms() & ALARM_OVERPRESSURE);
}

void test_occlusion_pauses_manual_run() {
    Harness h;
    h.pressurePresent = true;
    setLimit(h, 20);
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.run(20);   // normal: ≈ 2 kPa
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
    h.plant.setOcclusion(true);
    h.run(15);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::PAUSED), static_cast<int>(h.rm.state()));
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_OVERPRESSURE);
    TEST_ASSERT_EQUAL_FLOAT(0, h.out.command);
    // Still occluded → pressure can't fall below the limit while the line is
    // pressurised… but with the motor stopped it bleeds down; clear the
    // occlusion and resume once the pressure is back under the limit.
    h.plant.setOcclusion(false);
    h.run(10);
    TEST_ASSERT_NULL(h.rm.resume(h.t, true, false));
}

void test_resume_refused_while_high() {
    Harness h;
    h.pressurePresent = true;
    setLimit(h, 20);
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.plant.setOcclusion(true);
    h.run(15);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::PAUSED), static_cast<int>(h.rm.state()));
    h.pressureZero -= 50;   // pretend the line is still pressurised
    h.run(2);
    TEST_ASSERT_NOT_NULL(h.rm.resume(h.t, true, false));
    TEST_ASSERT_NOT_NULL(h.rm.start(manual(5), h.t, true, true));
}

void test_stop_action_and_calibration_abort() {
    Harness h;
    h.pressurePresent = true;
    setLimit(h, 20, true);
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.plant.setOcclusion(true);
    h.run(15);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    h.plant.setOcclusion(false);
    h.run(15);
    SweepConfig c;
    c.nCommands = linearCommands(5, 10, 2, c.commands);   // first point pumps, so pressure can build
    c.zeroFirst = false;
    c.settleS = 5;
    c.averageS = 5;
    TEST_ASSERT_NULL(h.rm.startCalibration(c, true, h.t));
    h.run(3);
    h.plant.setOcclusion(true);
    h.run(20);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::IDLE), static_cast<int>(h.rm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(SweepPhase::ABORTED), static_cast<int>(h.rm.sweep().phase()));
}

void test_pressure_sensor_fault_fails_closed_only_when_protecting() {
    Harness h;
    h.pressurePresent = true;
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.pressureOk = false;
    h.run(3);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));   // no limit set
    TEST_ASSERT_TRUE(h.rm.alarms() & ALARM_PRESSURE_SENSOR);
    setLimit(h, 20);
    h.run(3);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::PAUSED), static_cast<int>(h.rm.state()));
}

void test_saturation_counts_as_overpressure() {
    Harness h(100);
    h.pressurePresent = true;
    setLimit(h, 60);
    TEST_ASSERT_NULL(h.rm.start(manual(40), 0, true, true));   // 800 µL/min
    h.plant.setOcclusion(true);   // 800 × 0.02 × 40 = 640 kPa ≫ 172 kPa full scale
    h.run(15);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::PAUSED), static_cast<int>(h.rm.state()));
}

void test_debounce_ignores_short_spike() {
    Harness h;
    h.pressurePresent = true;
    setLimit(h, 20);
    TEST_ASSERT_NULL(h.rm.start(manual(5), 0, true, true));
    h.run(10);
    h.pressureZero -= 50;   // 0.5 s spike
    h.run(0.5f);
    h.pressureZero += 50;
    h.run(3);
    TEST_ASSERT_EQUAL(static_cast<int>(ChannelState::RUNNING), static_cast<int>(h.rm.state()));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_transfer_function_datasheet_example);
    RUN_TEST(test_transfer_function_0025pa_endpoints);
    RUN_TEST(test_status_decoding);
    RUN_TEST(test_garbled_frames_rejected);
    RUN_TEST(test_sim_pressure_tracks_flow_and_occlusion);
    RUN_TEST(test_no_limit_no_action);
    RUN_TEST(test_occlusion_pauses_manual_run);
    RUN_TEST(test_resume_refused_while_high);
    RUN_TEST(test_stop_action_and_calibration_abort);
    RUN_TEST(test_pressure_sensor_fault_fails_closed_only_when_protecting);
    RUN_TEST(test_saturation_counts_as_overpressure);
    RUN_TEST(test_debounce_ignores_short_spike);
    return UNITY_END();
}
