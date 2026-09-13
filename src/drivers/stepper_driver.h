#pragma once
// =============================================================================
// stepper_driver.h — Low-level STEP/DIR pulse generation
//
// Uses the ESP32's LEDC (hardware PWM) peripheral to generate the step pulse
// train. This is non-blocking: once you call setStepFrequency(), the hardware
// keeps pulsing on its own without CPU intervention, which matters because
// your control loop task needs to keep running on schedule regardless.
//
// This class only knows about "pulses per second" and direction — it has no
// concept of flow rate or mL. That conversion belongs in flow_controller.
// =============================================================================

#include <Arduino.h>

class StepperDriver {
public:
    StepperDriver(gpio_num_t stepPin, gpio_num_t dirPin, gpio_num_t enablePin);

    // Call once from setup().
    void begin();

    // Set pulse frequency in Hz (steps/sec). 0 = motor stationary.
    // Safe to call repeatedly from the control loop.
    void setStepFrequency(float hz, bool forward);

    // Immediately stop pulsing (does not disable the driver).
    void stop();

    // Enable/disable the driver output stage (most drivers are active LOW enable).
    void setEnabled(bool enabled);

    float currentFrequencyHz() const { return currentHz_; }

private:
    gpio_num_t stepPin_;
    gpio_num_t dirPin_;
    gpio_num_t enablePin_;
    float currentHz_ = 0.0f;

    static constexpr int LEDC_CHANNEL   = 0;
    static constexpr int LEDC_TIMER_BIT = 10;   // resolution; duty fixed at 50%
    static constexpr int LEDC_BASE_FREQ_MIN = 1;
};
