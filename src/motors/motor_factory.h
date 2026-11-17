#pragma once
#include "config/hardware_config.h"
#include "motors/motor_driver.h"

// Creates (but does not begin()) the driver for a channel. `slot` is the
// channel index; it selects the step-engine timer and LEDC channels.
// In SIM_MODE every type is backed by SimMotor (no GPIO).
MotorDriver* createMotor(const ChannelHw& hw, int slot);
