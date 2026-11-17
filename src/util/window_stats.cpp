#include "window_stats.h"
#include <cmath>

void WindowStats::configure(float windowS) {
    if (!(windowS > 0.05f)) windowS = 0.05f;
    windowS_ = windowS;
    bucketS_ = windowS / kBuckets;
    reset();
}

void WindowStats::reset() {
    for (auto& b : buckets_) b = Bucket{};
    head_ = 0;
    used_ = 0;
    haveRef_ = false;
    ref_ = 0;
}

void WindowStats::expire(float nowS) {
    // Drop buckets whose newest sample is older than the window.
    while (used_ > 1) {
        const int oldest = (head_ - used_ + 1 + kBuckets) % kBuckets;
        if (nowS - buckets_[oldest].t1 > windowS_) {
            buckets_[oldest] = Bucket{};
            --used_;
        } else {
            break;
        }
    }
}

void WindowStats::add(float tS, float x) {
    if (!std::isfinite(x)) return;
    if (!haveRef_) {
        ref_ = x;
        haveRef_ = true;
    }
    if (used_ == 0) {
        used_ = 1;
        buckets_[head_] = Bucket{};
        buckets_[head_].t0 = tS;
    } else if (tS - buckets_[head_].t0 >= bucketS_) {
        head_ = (head_ + 1) % kBuckets;
        buckets_[head_] = Bucket{};
        buckets_[head_].t0 = tS;
        if (used_ < kBuckets) ++used_;
    }
    Bucket& b = buckets_[head_];
    const double d = static_cast<double>(x) - ref_;
    b.sum += d;
    b.sumsq += d * d;
    b.n += 1;
    b.t1 = tS;
    expire(tS);
}

uint32_t WindowStats::count() const {
    uint32_t n = 0;
    for (int i = 0; i < used_; ++i) n += buckets_[(head_ - i + kBuckets) % kBuckets].n;
    return n;
}

float WindowStats::mean() const {
    double s = 0;
    uint32_t n = 0;
    for (int i = 0; i < used_; ++i) {
        const Bucket& b = buckets_[(head_ - i + kBuckets) % kBuckets];
        s += b.sum;
        n += b.n;
    }
    if (n == 0) return NAN;
    return static_cast<float>(ref_ + s / n);
}

float WindowStats::stddev() const {
    double s = 0, ss = 0;
    uint32_t n = 0;
    for (int i = 0; i < used_; ++i) {
        const Bucket& b = buckets_[(head_ - i + kBuckets) % kBuckets];
        s += b.sum;
        ss += b.sumsq;
        n += b.n;
    }
    if (n < 2) return 0.0f;
    const double var = (ss - s * s / n) / (n - 1);
    return var > 0 ? static_cast<float>(std::sqrt(var)) : 0.0f;
}

float WindowStats::slopePerS() const {
    // Least squares through bucket means vs bucket mid-times.
    double st = 0, sy = 0, stt = 0, sty = 0;
    int k = 0;
    for (int i = 0; i < used_; ++i) {
        const Bucket& b = buckets_[(head_ - i + kBuckets) % kBuckets];
        if (b.n == 0) continue;
        const double t = 0.5 * (b.t0 + b.t1);
        const double y = b.sum / b.n;
        st += t;
        sy += y;
        stt += t * t;
        sty += t * y;
        ++k;
    }
    if (k < 3) return 0.0f;
    const double den = k * stt - st * st;
    if (std::fabs(den) < 1e-12) return 0.0f;
    return static_cast<float>((k * sty - st * sy) / den);
}

float WindowStats::spanS() const {
    if (used_ == 0) return 0.0f;
    const Bucket& newest = buckets_[head_];
    const Bucket& oldest = buckets_[(head_ - used_ + 1 + kBuckets) % kBuckets];
    return newest.t1 - oldest.t0;
}

float Ema::update(float x, float dtS) {
    if (!std::isfinite(x)) return y_;
    if (!init_) {
        y_ = x;
        init_ = true;
        return y_;
    }
    const float a = (tauS_ <= 0.0f) ? 1.0f : 1.0f - std::exp(-dtS / tauS_);
    y_ += a * (x - y_);
    return y_;
}
