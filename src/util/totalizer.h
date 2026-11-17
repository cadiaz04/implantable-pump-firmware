#pragma once
// Volume totalizer: trapezoidal integration of a flow (µL/min) over time.
class Totalizer {
public:
    void reset() { volumeUl_ = 0; havePrev_ = false; }
    // Adds flow sample at time tS. Gaps longer than maxGapS are not bridged
    // (the integrator restarts) so a dropped sensor doesn't fake volume.
    void add(float tS, float flowUlMin, float maxGapS = 1.0f) {
        if (havePrev_) {
            const float dt = tS - prevT_;
            if (dt > 0 && dt <= maxGapS) volumeUl_ += 0.5f * (flowUlMin + prevFlow_) * dt / 60.0f;
        }
        prevT_ = tS;
        prevFlow_ = flowUlMin;
        havePrev_ = true;
    }
    // Break the trapezoid chain (e.g. when integration pauses).
    void breakChain() { havePrev_ = false; }
    double volumeUl() const { return volumeUl_; }
    void setVolumeUl(double v) { volumeUl_ = v; }

private:
    double volumeUl_ = 0;
    float prevT_ = 0, prevFlow_ = 0;
    bool havePrev_ = false;
};
