#pragma once
// Hardware configuration: which driver and pins each channel uses, and its
// flow-sensor bus. Pure struct (no Arduino) — persisted by settings_store.
#include "config.h"
#include "motors/motor_types.h"

enum class SensorType : uint8_t { NONE = 0, SLF3S = 1 };
// Pressure sensor on the same I²C bus as the channel's flow sensor.
enum class PressureType : uint8_t { NONE = 0, MPRLS = 1 };

struct ChannelHw {
    DriverType driver = DriverType::ULN2003;
    int pins[4] = {-1, -1, -1, -1};   // meaning depends on driver (see pinLabel)
    bool enActiveLow = true;          // STEP/DIR EN polarity
    SensorType sensor = SensorType::SLF3S;
    PressureType pressure = PressureType::NONE;
    int sda = -1, scl = -1;   // I²C bus shared by the flow and pressure sensors
    bool usesBus() const { return sensor != SensorType::NONE || pressure != PressureType::NONE; }
};

struct HardwareConfig {
    ChannelHw ch[MAX_CHANNELS];
};

HardwareConfig defaultHardwareConfig();
// Fills the default pins for driver type t on channel idx.
void applyDefaultPins(ChannelHw& hw, int idx, DriverType t);
const char* pinLabel(DriverType t, int slot);   // e.g. "IN1", "STEP", "EN"
