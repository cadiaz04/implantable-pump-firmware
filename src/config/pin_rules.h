#pragma once
// =============================================================================
// pin_rules — validates GPIO choices for the classic ESP32 (WROOM-32).
// Pure logic, unit tested natively.
//
//   * non-existent GPIOs (20, 24, 28-31, >39)           → illegal
//   * flash pins 6-11                                   → illegal
//   * UART0 TX/RX 1, 3 (USB serial)                     → illegal
//   * strapping pins 0, 2, 5, 12, 15                    → illegal (boot risk)
//   * input-only 34-39                                  → illegal as outputs / I²C
//   * any pin used twice across all channels + buses    → conflict
// =============================================================================
#include "hardware_config.h"

enum class PinUse : uint8_t { DIGITAL_OUT, I2C, ADC_INPUT };

// Returns nullptr if OK, otherwise a short reason ("flash pin", ...).
const char* checkPin(int gpio, PinUse use);

struct PinError {
    int channel;      // 0-based, -1 = global
    char field[12];   // e.g. "IN1", "SDA"
    int gpio;
    char reason[48];
};

// Validates the whole configuration. Returns number of errors written
// (up to maxErrors).
int validateHardware(const HardwareConfig& cfg, PinError* errors, int maxErrors);
