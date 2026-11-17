#pragma once
// =============================================================================
// curve_fit — flow = f(command) fits and their inverses.
//
//   LINEAR     flow = c0 + c1·x            (default)
//   QUADRATIC  flow = c0 + c1·x + c2·x²
//   LOOKUP     monotone piecewise-linear table (duplicates averaged,
//              non-monotone data repaired by pool-adjacent-violators and
//              flagged with nonMonotone = true)
//
// The inverse (command for a target flow) is evaluated only where the fit
// is increasing and is always clamped to the caller's [lo, hi] command range.
// Pure logic, unit tested natively.
// =============================================================================
#include <cstdint>

enum class FitType : uint8_t { LINEAR = 0, QUADRATIC = 1, LOOKUP = 2 };
const char* fitTypeId(FitType t);          // "linear" | "quadratic" | "lookup"
bool fitTypeFromId(const char* id, FitType& out);

constexpr int MAX_LOOKUP_POINTS = 32;

struct CurveFit {
    FitType type = FitType::LINEAR;
    bool ok = false;
    float c[3] = {0, 0, 0};
    int n = 0;                              // lookup table size
    float lx[MAX_LOOKUP_POINTS] = {};
    float ly[MAX_LOOKUP_POINTS] = {};
    float xMin = 0, xMax = 0;               // data domain
    float r2 = 0, rmse = 0, maxErrPct = 0;
    bool nonMonotone = false;

    float eval(float x) const;
    // Command in [lo, hi] that yields `flow` (bisection on the increasing part).
    float inverse(float flow, float lo, float hi) const;
    // True if the fit increases somewhere inside [lo, hi] — required for use
    // as a feed-forward model.
    bool usableIn(float lo, float hi) const;
};

// Fits n points. Returns false and sets *err (static string) on failure.
bool fitCurve(const float* x, const float* y, int n, FitType type, CurveFit& out, const char** err);

// Fills residual statistics (r2, rmse, maxErrPct) of `fit` on the data.
void computeFitStats(const float* x, const float* y, int n, CurveFit& fit);
