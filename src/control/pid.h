#pragma once
// PID with derivative-on-measurement and two anti-windup mechanisms:
//   * the integral term is clamped to [iMin, iMax]
//   * conditional integration: the integrator does not grow further in a
//     direction in which the output (or the downstream actuator, via
//     `satDir`) is already saturated.
// Pure logic, unit tested natively.

struct PidConfig {
    float kp = 0, ki = 0, kd = 0;
    float outMin = -1e9f, outMax = 1e9f;
    float iMin = -1e9f, iMax = 1e9f;
};

class Pid {
public:
    void configure(const PidConfig& c) { cfg_ = c; }
    const PidConfig& config() const { return cfg_; }
    void reset(float integral = 0) {
        integral_ = integral;
        havePrev_ = false;
    }
    // satDir: +1 if the actuator is saturated high, −1 low, 0 otherwise.
    float update(float setpoint, float measurement, float dtS, int satDir = 0);
    float integral() const { return integral_; }
    float lastP() const { return p_; }
    float lastD() const { return d_; }

private:
    PidConfig cfg_;
    float integral_ = 0;
    float prevMeas_ = 0;
    bool havePrev_ = false;
    float p_ = 0, d_ = 0;
};
