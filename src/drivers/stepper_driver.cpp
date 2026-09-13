#include "stepper_driver.h"

StepperDriver::StepperDriver(gpio_num_t stepPin, gpio_num_t dirPin, gpio_num_t enablePin)
    : stepPin_(stepPin), dirPin_(dirPin), enablePin_(enablePin) {}

void StepperDriver::begin() {
    pinMode(dirPin_, OUTPUT);
    pinMode(enablePin_, OUTPUT);
    setEnabled(false);   // start disabled — explicit setEnabled(true) required before motion

    ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ_MIN, LEDC_TIMER_BIT);
    ledcAttachPin(stepPin_, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, 0);   // 0 duty = no pulses yet
}

void StepperDriver::setStepFrequency(float hz, bool forward) {
    digitalWrite(dirPin_, forward ? HIGH : LOW);

    if (hz <= 0.0f) {
        stop();
        return;
    }

    // Reconfigure the LEDC timer frequency, keep duty at 50% for a clean step pulse.
    ledcSetup(LEDC_CHANNEL, static_cast<uint32_t>(hz), LEDC_TIMER_BIT);
    ledcAttachPin(stepPin_, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, (1 << (LEDC_TIMER_BIT - 1)));   // 50% duty
    currentHz_ = hz;
}

void StepperDriver::stop() {
    ledcWrite(LEDC_CHANNEL, 0);
    currentHz_ = 0.0f;
}

void StepperDriver::setEnabled(bool enabled) {
    // Most STEP/DIR drivers (A4988, DRV8825, TMC2209 legacy mode) use active-LOW enable.
    digitalWrite(enablePin_, enabled ? LOW : HIGH);
}
