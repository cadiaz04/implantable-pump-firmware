#include "curve_fit.h"
#include <cmath>
#include <cstring>

const char* fitTypeId(FitType t) {
    switch (t) {
        case FitType::QUADRATIC: return "quadratic";
        case FitType::LOOKUP: return "lookup";
        default: return "linear";
    }
}

bool fitTypeFromId(const char* id, FitType& out) {
    if (!id) return false;
    if (!strcmp(id, "linear")) out = FitType::LINEAR;
    else if (!strcmp(id, "quadratic")) out = FitType::QUADRATIC;
    else if (!strcmp(id, "lookup")) out = FitType::LOOKUP;
    else return false;
    return true;
}

float CurveFit::eval(float x) const {
    if (type != FitType::LOOKUP) return c[0] + c[1] * x + c[2] * x * x;
    if (n <= 0) return 0.0f;
    if (n == 1) return ly[0];
    int i = 0;
    if (x <= lx[0]) i = 0;
    else if (x >= lx[n - 1]) i = n - 2;
    else {
        while (i < n - 2 && x > lx[i + 1]) ++i;
    }
    const float dx = lx[i + 1] - lx[i];
    if (dx <= 0) return ly[i];
    float yv = ly[i] + (ly[i + 1] - ly[i]) * (x - lx[i]) / dx;
    // Outside the table extrapolate along the end segment.
    return yv;
}

static void increasingRegion(const CurveFit& f, float& lo, float& hi) {
    if (f.type == FitType::QUADRATIC && f.c[2] != 0.0f) {
        const float vertex = -f.c[1] / (2.0f * f.c[2]);
        if (f.c[2] > 0) {
            if (vertex > lo) lo = vertex;
        } else {
            if (vertex < hi) hi = vertex;
        }
    }
}

bool CurveFit::usableIn(float lo, float hi) const {
    if (!ok || !(hi > lo)) return false;
    switch (type) {
        case FitType::LINEAR: return c[1] > 0;
        case FitType::QUADRATIC: {
            float a = lo, b = hi;
            increasingRegion(*this, a, b);
            return b > a && eval(b) > eval(a);
        }
        case FitType::LOOKUP: return n >= 2 && ly[n - 1] > ly[0];
    }
    return false;
}

float CurveFit::inverse(float flow, float lo, float hi) const {
    if (!(hi > lo)) return lo;
    float a = lo, b = hi;
    if (type == FitType::LINEAR) {
        if (c[1] <= 0) return lo;
        const float x = (flow - c[0]) / c[1];
        return x < lo ? lo : (x > hi ? hi : x);
    }
    increasingRegion(*this, a, b);
    if (!(b > a)) return lo;
    const float fa = eval(a), fb = eval(b);
    if (flow <= fa) return a;
    if (flow >= fb) return b;
    for (int it = 0; it < 48; ++it) {
        const float m = 0.5f * (a + b);
        if (eval(m) < flow) a = m;
        else b = m;
    }
    return 0.5f * (a + b);
}

void computeFitStats(const float* x, const float* y, int n, CurveFit& fit) {
    double mean = 0, ymax = 0;
    for (int i = 0; i < n; ++i) {
        mean += y[i];
        if (std::fabs(y[i]) > ymax) ymax = std::fabs(y[i]);
    }
    mean /= (n > 0 ? n : 1);
    double ssRes = 0, ssTot = 0, maxPct = 0;
    // Points near zero flow would dominate a %-error metric; ignore those
    // below 2% of the largest flow.
    const double floor = 0.02 * ymax;
    for (int i = 0; i < n; ++i) {
        const double r = y[i] - fit.eval(x[i]);
        ssRes += r * r;
        ssTot += (y[i] - mean) * (y[i] - mean);
        if (std::fabs(y[i]) > floor && std::fabs(y[i]) > 1e-9) {
            const double p = std::fabs(r) / std::fabs(y[i]) * 100.0;
            if (p > maxPct) maxPct = p;
        }
    }
    fit.r2 = ssTot > 0 ? static_cast<float>(1.0 - ssRes / ssTot) : (ssRes < 1e-12 ? 1.0f : 0.0f);
    fit.rmse = n > 0 ? static_cast<float>(std::sqrt(ssRes / n)) : 0.0f;
    fit.maxErrPct = static_cast<float>(maxPct);
}

namespace {

// Solves the 3x3 system A·s = b (Gaussian elimination, partial pivoting).
bool solve3(double A[3][3], double b[3], double s[3]) {
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        for (int r = col + 1; r < 3; ++r)
            if (std::fabs(A[r][col]) > std::fabs(A[piv][col])) piv = r;
        if (std::fabs(A[piv][col]) < 1e-12) return false;
        if (piv != col) {
            for (int k = 0; k < 3; ++k) {
                double t = A[col][k];
                A[col][k] = A[piv][k];
                A[piv][k] = t;
            }
            double t = b[col];
            b[col] = b[piv];
            b[piv] = t;
        }
        for (int r = col + 1; r < 3; ++r) {
            const double f = A[r][col] / A[col][col];
            for (int k = col; k < 3; ++k) A[r][k] -= f * A[col][k];
            b[r] -= f * b[col];
        }
    }
    for (int r = 2; r >= 0; --r) {
        double acc = b[r];
        for (int k = r + 1; k < 3; ++k) acc -= A[r][k] * s[k];
        s[r] = acc / A[r][r];
    }
    return true;
}

int distinctX(const float* x, int n) {
    int d = 0;
    for (int i = 0; i < n; ++i) {
        bool seen = false;
        for (int j = 0; j < i; ++j)
            if (std::fabs(x[j] - x[i]) <= 1e-6f * (1.0f + std::fabs(x[i]))) seen = true;
        if (!seen) ++d;
    }
    return d;
}

}  // namespace

bool fitCurve(const float* x, const float* y, int n, FitType type, CurveFit& out, const char** err) {
    static const char* kNone = "";
    if (err) *err = kNone;
    out = CurveFit{};
    out.type = type;
    if (n < 2) {
        if (err) *err = "need at least 2 valid points";
        return false;
    }
    float xmin = x[0], xmax = x[0];
    for (int i = 1; i < n; ++i) {
        if (x[i] < xmin) xmin = x[i];
        if (x[i] > xmax) xmax = x[i];
    }
    out.xMin = xmin;
    out.xMax = xmax;
    const int dx = distinctX(x, n);

    // Center/scale x for numerical conditioning.
    double xm = 0;
    for (int i = 0; i < n; ++i) xm += x[i];
    xm /= n;
    double xs = 0.5 * (xmax - xmin);
    if (xs <= 0) xs = 1;

    if (type == FitType::LINEAR) {
        if (dx < 2) {
            if (err) *err = "need at least 2 distinct commands";
            return false;
        }
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < n; ++i) {
            const double u = (x[i] - xm) / xs;
            sx += u;
            sy += y[i];
            sxx += u * u;
            sxy += u * y[i];
        }
        const double den = n * sxx - sx * sx;
        if (std::fabs(den) < 1e-12) {
            if (err) *err = "degenerate data";
            return false;
        }
        const double b1 = (n * sxy - sx * sy) / den;   // per scaled unit
        const double b0 = (sy - b1 * sx) / n;
        out.c[1] = static_cast<float>(b1 / xs);
        out.c[0] = static_cast<float>(b0 - b1 * xm / xs);
        out.c[2] = 0;
        out.ok = true;
    } else if (type == FitType::QUADRATIC) {
        if (dx < 3) {
            if (err) *err = "need at least 3 distinct commands";
            return false;
        }
        double A[3][3] = {}, b[3] = {}, s[3] = {};
        for (int i = 0; i < n; ++i) {
            const double u = (x[i] - xm) / xs;
            const double p[3] = {1, u, u * u};
            for (int r = 0; r < 3; ++r) {
                for (int k = 0; k < 3; ++k) A[r][k] += p[r] * p[k];
                b[r] += p[r] * y[i];
            }
        }
        if (!solve3(A, b, s)) {
            if (err) *err = "degenerate data";
            return false;
        }
        // y = s0 + s1·u + s2·u², u = (x − xm)/xs  → expand in x.
        const double k1 = s[1] / xs, k2 = s[2] / (xs * xs);
        out.c[0] = static_cast<float>(s[0] - k1 * xm + k2 * xm * xm);
        out.c[1] = static_cast<float>(k1 - 2 * k2 * xm);
        out.c[2] = static_cast<float>(k2);
        out.ok = true;
    } else {
        // Group equal commands, average flows, sort by command.
        float gx[MAX_LOOKUP_POINTS];
        double gsum[MAX_LOOKUP_POINTS];
        int gcnt[MAX_LOOKUP_POINTS];
        int g = 0;
        for (int i = 0; i < n; ++i) {
            int k = -1;
            for (int j = 0; j < g; ++j)
                if (std::fabs(gx[j] - x[i]) <= 1e-6f * (1.0f + std::fabs(x[i]))) k = j;
            if (k < 0) {
                if (g >= MAX_LOOKUP_POINTS) {
                    if (err) *err = "too many distinct commands for lookup";
                    return false;
                }
                k = g++;
                gx[k] = x[i];
                gsum[k] = 0;
                gcnt[k] = 0;
            }
            gsum[k] += y[i];
            gcnt[k] += 1;
        }
        if (g < 2) {
            if (err) *err = "need at least 2 distinct commands";
            return false;
        }
        // insertion sort by x
        for (int i = 1; i < g; ++i) {
            for (int j = i; j > 0 && gx[j - 1] > gx[j]; --j) {
                float tx = gx[j]; gx[j] = gx[j - 1]; gx[j - 1] = tx;
                double ts = gsum[j]; gsum[j] = gsum[j - 1]; gsum[j - 1] = ts;
                int tc = gcnt[j]; gcnt[j] = gcnt[j - 1]; gcnt[j - 1] = tc;
            }
        }
        // Pool-adjacent-violators → non-decreasing y.
        double bv[MAX_LOOKUP_POINTS];
        double bw[MAX_LOOKUP_POINTS];
        int bstart[MAX_LOOKUP_POINTS];
        int nb = 0;
        for (int i = 0; i < g; ++i) {
            bv[nb] = gsum[i] / gcnt[i];
            bw[nb] = gcnt[i];
            bstart[nb] = i;
            ++nb;
            while (nb > 1 && bv[nb - 2] > bv[nb - 1]) {
                out.nonMonotone = true;
                const double w = bw[nb - 2] + bw[nb - 1];
                bv[nb - 2] = (bv[nb - 2] * bw[nb - 2] + bv[nb - 1] * bw[nb - 1]) / w;
                bw[nb - 2] = w;
                --nb;
            }
        }
        for (int bi = 0; bi < nb; ++bi) {
            const int end = (bi + 1 < nb) ? bstart[bi + 1] : g;
            for (int i = bstart[bi]; i < end; ++i) out.ly[i] = static_cast<float>(bv[bi]);
        }
        for (int i = 0; i < g; ++i) out.lx[i] = gx[i];
        out.n = g;
        out.ok = true;
    }
    computeFitStats(x, y, n, out);
    return true;
}
