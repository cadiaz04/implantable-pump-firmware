#pragma once
// =============================================================================
// WindowStats — rolling mean / standard deviation / slope over a TIME window.
//
// Low flows need long averaging windows (minutes), which at 50 Hz would be
// thousands of samples. Instead samples are folded into kBuckets time buckets
// (bucket length = window / kBuckets), so memory is fixed (~1.5 KB) no matter
// how long the window is. Mean and std are exact over the samples held in the
// buckets; slope is a least-squares fit through the bucket means.
// =============================================================================
#include <cstdint>

class WindowStats {
public:
    static constexpr int kBuckets = 60;

    explicit WindowStats(float windowS = 10.0f) { configure(windowS); }

    void configure(float windowS);
    void reset();
    void add(float tS, float x);

    float windowS() const { return windowS_; }
    uint32_t count() const;
    float mean() const;
    float stddev() const;          // sample std dev of the raw samples
    float slopePerS() const;       // d(mean)/dt in units/s (0 if < 3 buckets)
    float spanS() const;           // time covered by the data currently held

private:
    struct Bucket {
        double sum = 0, sumsq = 0;
        uint32_t n = 0;
        float t0 = 0, t1 = 0;
    };
    void expire(float nowS);

    float windowS_ = 10.0f;
    float bucketS_ = 0.2f;
    Bucket buckets_[kBuckets];
    int head_ = 0;      // index of the bucket currently being filled
    int used_ = 0;      // number of buckets in use (including current)
    bool haveRef_ = false;
    double ref_ = 0;    // reference value subtracted to reduce cancellation
};

// Exponential moving average with a time constant, robust to uneven dt.
class Ema {
public:
    explicit Ema(float tauS = 5.0f) : tauS_(tauS) {}
    void setTau(float tauS) { tauS_ = tauS; }
    void reset() { init_ = false; y_ = 0; }
    float update(float x, float dtS);
    float value() const { return y_; }
    bool initialized() const { return init_; }

private:
    float tauS_;
    float y_ = 0;
    bool init_ = false;
};
