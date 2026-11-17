#include "motor_factory.h"
#include "motors/dc_hbridge.h"
#include "motors/sim_motor.h"
#include "motors/stepper_drivers.h"

MotorDriver* createMotor(const ChannelHw& hw, int slot) {
#ifdef SIM_MODE
    if (hw.driver == DriverType::NONE) return nullptr;
    return new SimMotor(hw.driver);
#else
    switch (hw.driver) {
        case DriverType::ULN2003: return new StepperULN2003(slot, hw.pins);
        case DriverType::STEP_DIR: return new StepperStepDir(slot, hw.pins[0], hw.pins[1], hw.pins[2], hw.enActiveLow);
        case DriverType::DRV8871: return new DcDrv8871(slot, hw.pins[0], hw.pins[1]);
        case DriverType::PWM_DIR: return new DcPwmDir(slot, hw.pins[0], hw.pins[1], hw.pins[2]);
        default: return nullptr;
    }
#endif
}
