#pragma once
// =============================================================================
// sensor.h — generic sensor + signal model.
//
// Everything that produces numbers is a *signal* {id, name, unit, channel}.
// The UI builds plot series and CSV columns from the registry, so a new
// sensor (e.g. pressure) only needs a Sensor subclass registered in
// SensorRegistry — nothing else.
//
// Pure interface (no Arduino) so the control logic can be tested natively.
// =============================================================================
#include <cstdint>

struct SignalInfo {
    const char* id;      // stable key, e.g. "p1" — used in telemetry + CSV
    const char* name;    // human label
    const char* unit;    // "µL/min", "kPa", ...
    int8_t channel;      // pump channel it belongs to, -1 = global
};

enum class SensorStatus : uint8_t { ABSENT, INIT, OK, FAULT };
inline const char* sensorStatusName(SensorStatus s) {
    switch (s) {
        case SensorStatus::ABSENT: return "DISABLED";
        case SensorStatus::INIT: return "INIT";
        case SensorStatus::OK: return "OK";
        default: return "FAULT";
    }
}

struct SensorErrors {
    uint32_t crc = 0;
    uint32_t i2c = 0;
    uint32_t consecutive = 0;
    uint32_t reinits = 0;
};

class Sensor {
public:
    virtual ~Sensor() = default;
    virtual bool begin() = 0;
    // Called from the real-time task every tick. Returns true if a new sample
    // was produced. Implementations rate-limit themselves.
    virtual bool poll(uint32_t nowMs) = 0;
    virtual SensorStatus status() const = 0;
    virtual const SensorErrors& errors() const = 0;

    virtual int signalCount() const = 0;
    virtual SignalInfo signalInfo(int i) const = 0;
    virtual float signalValue(int i) const = 0;
};

// A flow sensor bound to a pump channel. Values are as measured (no zero
// offset applied — the channel owns the tare offset).
class FlowSensor : public Sensor {
public:
    virtual float flowUlMin() const = 0;
    virtual float tempC() const = 0;
    virtual uint16_t flags() const = 0;
    virtual bool airInLine() const = 0;
    virtual bool highFlow() const = 0;
    virtual uint32_t sampleTimeMs() const = 0;
};

// An absolute pressure sensor bound to a pump channel (e.g. Honeywell MPRLS).
// Values are absolute; the channel owns the tare (atmospheric) offset that
// turns them into gauge pressure.
class PressureSensor : public Sensor {
public:
    virtual float absKpa() const = 0;        // last decoded absolute pressure, kPa
    virtual bool saturated() const = 0;      // last frame flagged out of range
    virtual uint8_t statusByte() const = 0;  // raw sensor status for diagnostics
    virtual uint32_t sampleTimeMs() const = 0;
};
