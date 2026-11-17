#include <unity.h>
#include <cstring>
#include "config/pin_rules.h"
#include "motors/motor_types.h"

void setUp() {}
void tearDown() {}

void test_steps_per_second_conversion() {
    MotorParams uln;   // 28BYJ-48 defaults: 4096 half-steps/rev
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1024.0f, rpmToStepsPerSec(15, uln));
    MotorParams nema;
    nema.stepsPerRev = 200;
    nema.microstep = 16;
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 3200.0f, rpmToStepsPerSec(60, nema));
    nema.gearRatio = 5;
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 16000.0f, rpmToStepsPerSec(60, nema));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 60.0f, stepsPerSecToRpm(16000, nema));
}

void test_syringe_theoretical_flow() {
    // Ø 4.78 mm, 0.8 mm pitch → π/4·4.78²·0.8 ≈ 14.356 µL/rev
    TEST_ASSERT_FLOAT_WITHIN(0.01, 14.356f, syringeUlMinPerRpm(4.78f, 0.8f));
    TEST_ASSERT_EQUAL_FLOAT(0, syringeUlMinPerRpm(0, 1));
}

void test_ramp() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1.0f, rampToward(0, 10, 10, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 10.0f, rampToward(9.5f, 10, 10, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 9.0f, rampToward(10, 0, 10, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 7.0f, rampToward(0, 7, 0, 0.1f));   // accel 0 = no limit
}

void test_driver_ids_roundtrip() {
    const DriverType all[] = {DriverType::ULN2003, DriverType::STEP_DIR, DriverType::DRV8871, DriverType::PWM_DIR};
    for (DriverType t : all) TEST_ASSERT_EQUAL(static_cast<int>(t), static_cast<int>(driverTypeFromId(driverTypeId(t))));
    TEST_ASSERT_EQUAL(static_cast<int>(DriverType::NONE), static_cast<int>(driverTypeFromId("bogus")));
    TEST_ASSERT_EQUAL_STRING("Duty %", commandUnitFor(DriverType::DRV8871));
    TEST_ASSERT_EQUAL_STRING("RPM", commandUnitFor(DriverType::STEP_DIR));
}

void test_check_pin_rules() {
    TEST_ASSERT_NULL(checkPin(16, PinUse::DIGITAL_OUT));
    TEST_ASSERT_NULL(checkPin(33, PinUse::I2C));
    TEST_ASSERT_NOT_NULL(checkPin(6, PinUse::DIGITAL_OUT));    // flash
    TEST_ASSERT_NOT_NULL(checkPin(11, PinUse::DIGITAL_OUT));
    TEST_ASSERT_NOT_NULL(checkPin(0, PinUse::DIGITAL_OUT));    // strapping
    TEST_ASSERT_NOT_NULL(checkPin(12, PinUse::DIGITAL_OUT));
    TEST_ASSERT_NOT_NULL(checkPin(15, PinUse::DIGITAL_OUT));
    TEST_ASSERT_NOT_NULL(checkPin(34, PinUse::DIGITAL_OUT));   // input only
    TEST_ASSERT_NOT_NULL(checkPin(39, PinUse::I2C));
    TEST_ASSERT_NOT_NULL(checkPin(20, PinUse::DIGITAL_OUT));   // doesn't exist
    TEST_ASSERT_NOT_NULL(checkPin(1, PinUse::DIGITAL_OUT));    // UART0
    TEST_ASSERT_NOT_NULL(checkPin(40, PinUse::DIGITAL_OUT));
    TEST_ASSERT_NULL(checkPin(36, PinUse::ADC_INPUT));
    TEST_ASSERT_NOT_NULL(checkPin(25, PinUse::ADC_INPUT));   // ADC2
}

void test_default_config_is_valid() {
    HardwareConfig c = defaultHardwareConfig();
    PinError errs[8];
    TEST_ASSERT_EQUAL(0, validateHardware(c, errs, 8));
}

void test_conflicts_detected() {
    HardwareConfig c = defaultHardwareConfig();
    c.ch[1].pins[0] = 16;   // Ch2 IN1 = Ch1 IN1
    PinError errs[8];
    const int n = validateHardware(c, errs, 8);
    TEST_ASSERT_EQUAL(1, n);
    TEST_ASSERT_EQUAL(1, errs[0].channel);
    TEST_ASSERT_EQUAL(16, errs[0].gpio);
    TEST_ASSERT_NOT_NULL(strstr(errs[0].reason, "conflicts"));
}

void test_conflict_with_i2c_and_illegal_pin() {
    HardwareConfig c = defaultHardwareConfig();
    c.ch[0].pins[3] = 21;   // collides with Ch1 SDA
    c.ch[1].pins[1] = 7;    // flash
    PinError errs[8];
    TEST_ASSERT_EQUAL(2, validateHardware(c, errs, 8));
}

void test_missing_required_pin_and_optional_en() {
    HardwareConfig c = defaultHardwareConfig();
    applyDefaultPins(c.ch[0], 0, DriverType::STEP_DIR);
    c.ch[0].pins[2] = -1;   // EN optional
    PinError errs[8];
    TEST_ASSERT_EQUAL(0, validateHardware(c, errs, 8));
    c.ch[0].pins[0] = -1;   // STEP required
    TEST_ASSERT_EQUAL(1, validateHardware(c, errs, 8));
}

void test_same_sda_scl_rejected() {
    HardwareConfig c = defaultHardwareConfig();
    c.ch[0].scl = c.ch[0].sda;
    PinError errs[8];
    TEST_ASSERT_TRUE(validateHardware(c, errs, 8) >= 1);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_steps_per_second_conversion);
    RUN_TEST(test_syringe_theoretical_flow);
    RUN_TEST(test_ramp);
    RUN_TEST(test_driver_ids_roundtrip);
    RUN_TEST(test_check_pin_rules);
    RUN_TEST(test_default_config_is_valid);
    RUN_TEST(test_conflicts_detected);
    RUN_TEST(test_conflict_with_i2c_and_illegal_pin);
    RUN_TEST(test_missing_required_pin_and_optional_en);
    RUN_TEST(test_same_sda_scl_rejected);
    return UNITY_END();
}
