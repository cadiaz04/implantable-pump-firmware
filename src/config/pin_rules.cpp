#include "pin_rules.h"
#include <cstdio>
#include <cstring>

const char* checkPin(int gpio, PinUse use) {
    if (gpio < 0 || gpio > 39) return "not a GPIO";
    if (gpio == 20 || gpio == 24 || (gpio >= 28 && gpio <= 31)) return "not a GPIO on ESP32";
    if (gpio >= 6 && gpio <= 11) return "flash pin (6-11)";
    if (gpio == 1 || gpio == 3) return "UART0 / USB serial";
    if (gpio == 0 || gpio == 2 || gpio == 5 || gpio == 12 || gpio == 15) return "strapping pin";
    if (use == PinUse::ADC_INPUT) {
        if (gpio < 32) return "not ADC1 (ADC2 unusable with Wi-Fi)";
        return nullptr;
    }
    if (gpio >= 34) return "input-only pin";
    return nullptr;
}

const char* pinLabel(DriverType t, int slot) {
    static const char* uln[] = {"IN1", "IN2", "IN3", "IN4"};
    static const char* sd[] = {"STEP", "DIR", "EN"};
    static const char* drv[] = {"IN1", "IN2"};
    static const char* pd[] = {"PWM", "DIR", "EN"};
    if (slot < 0 || slot >= maxPinCount(t)) return "-";
    switch (t) {
        case DriverType::ULN2003: return uln[slot];
        case DriverType::STEP_DIR: return sd[slot];
        case DriverType::DRV8871: return drv[slot];
        case DriverType::PWM_DIR: return pd[slot];
        default: return "-";
    }
}

void applyDefaultPins(ChannelHw& hw, int idx, DriverType t) {
    const DefaultChannelPins& d = DEFAULT_PINS[idx];
    hw.driver = t;
    for (int& p : hw.pins) p = -1;
    switch (t) {
        case DriverType::ULN2003:
            for (int i = 0; i < 4; ++i) hw.pins[i] = d.uln[i];
            break;
        case DriverType::STEP_DIR:
        case DriverType::PWM_DIR:
            for (int i = 0; i < 3; ++i) hw.pins[i] = d.stepDirEn[i];
            break;
        case DriverType::DRV8871:
            hw.pins[0] = d.dcIn[0];
            hw.pins[1] = d.dcIn[1];
            break;
        default: break;
    }
    hw.sda = d.sda;
    hw.scl = d.scl;
}

HardwareConfig defaultHardwareConfig() {
    HardwareConfig c;
    // Ch 1: 28BYJ-48 on a ULN2003 board; Ch 2: DC pump on a DRV8871.
    applyDefaultPins(c.ch[0], 0, DriverType::ULN2003);
    if (MAX_CHANNELS > 1) applyDefaultPins(c.ch[1], 1, DriverType::DRV8871);
    for (auto& ch : c.ch) {
        ch.sensor = SensorType::SLF3S;
        ch.enActiveLow = true;
    }
    // One MicroPressure sensor bought so far: on Ch 1's bus.
    c.ch[0].pressure = PressureType::MPRLS;
    return c;
}

namespace {
struct Used {
    int gpio;
    int channel;
    const char* field;
};

void addError(PinError* errors, int maxErrors, int& n, int ch, const char* field, int gpio, const char* reason) {
    if (n >= maxErrors) {
        ++n;
        return;
    }
    PinError& e = errors[n++];
    e.channel = ch;
    snprintf(e.field, sizeof(e.field), "%s", field);
    e.gpio = gpio;
    snprintf(e.reason, sizeof(e.reason), "%s", reason);
}
}  // namespace

int validateHardware(const HardwareConfig& cfg, PinError* errors, int maxErrors) {
    int n = 0;
    Used used[MAX_CHANNELS * 6];
    int nUsed = 0;

    auto claim = [&](int ch, const char* field, int gpio, PinUse use) {
        if (const char* why = checkPin(gpio, use)) {
            addError(errors, maxErrors, n, ch, field, gpio, why);
            return;
        }
        for (int i = 0; i < nUsed; ++i) {
            if (used[i].gpio == gpio) {
                char msg[48];
                snprintf(msg, sizeof(msg), "conflicts with Ch%d %s", used[i].channel + 1, used[i].field);
                addError(errors, maxErrors, n, ch, field, gpio, msg);
                return;
            }
        }
        used[nUsed++] = {gpio, ch, field};
    };

    for (int c = 0; c < MAX_CHANNELS; ++c) {
        const ChannelHw& hw = cfg.ch[c];
        if (hw.driver == DriverType::NONE) continue;
        if (familyOf(hw.driver) == DriverFamily::NONE) {
            addError(errors, maxErrors, n, c, "driver", -1, "unknown driver type");
            continue;
        }
        const int req = requiredPinCount(hw.driver);
        const int mx = maxPinCount(hw.driver);
        for (int s = 0; s < mx; ++s) {
            const int g = hw.pins[s];
            if (g < 0) {
                if (s < req) addError(errors, maxErrors, n, c, pinLabel(hw.driver, s), g, "required pin not set");
                continue;   // optional EN left unconnected
            }
            claim(c, pinLabel(hw.driver, s), g, PinUse::DIGITAL_OUT);
        }
        if (hw.usesBus()) {
            if (hw.sda == hw.scl) {
                addError(errors, maxErrors, n, c, "SDA", hw.sda, "SDA and SCL are the same pin");
                continue;
            }
            claim(c, "SDA", hw.sda, PinUse::I2C);
            claim(c, "SCL", hw.scl, PinUse::I2C);
        }
    }
    return n;
}
