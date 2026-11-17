#include <unity.h>
#include <cmath>
#include "calibration/curve_fit.h"

void setUp() {}
void tearDown() {}

void test_linear_exact() {
    const float x[] = {0, 2, 4, 6, 8, 10};
    float y[6];
    for (int i = 0; i < 6; ++i) y[i] = 5.0f + 25.0f * x[i];
    CurveFit f;
    const char* err;
    TEST_ASSERT_TRUE(fitCurve(x, y, 6, FitType::LINEAR, f, &err));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 5.0f, f.c[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 25.0f, f.c[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, f.r2);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, f.rmse);
    // inverse
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.3f, f.inverse(62.5f, 0, 10));
    // clamped to limits
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 10.0f, f.inverse(10000.0f, 0, 10));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, f.inverse(-50.0f, 0, 10));
}

void test_linear_noisy_known_data() {
    // Hand-computed: x = 1..5, y = 2.1, 3.9, 6.2, 7.8, 10.1 → slope 1.99, intercept 0.05
    const float x[] = {1, 2, 3, 4, 5};
    const float y[] = {2.1f, 3.9f, 6.2f, 7.8f, 10.1f};
    CurveFit f;
    TEST_ASSERT_TRUE(fitCurve(x, y, 5, FitType::LINEAR, f, nullptr));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.99f, f.c[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.05f, f.c[0]);
    TEST_ASSERT_TRUE(f.r2 > 0.99f && f.r2 < 1.0f);
    TEST_ASSERT_TRUE(f.maxErrPct > 0);
}

void test_quadratic_exact() {
    const float x[] = {10, 20, 30, 40, 50, 60, 70};
    float y[7];
    for (int i = 0; i < 7; ++i) y[i] = -40.0f + 6.0f * x[i] + 0.02f * x[i] * x[i];
    CurveFit f;
    TEST_ASSERT_TRUE(fitCurve(x, y, 7, FitType::QUADRATIC, f, nullptr));
    TEST_ASSERT_FLOAT_WITHIN(1e-2, -40.0f, f.c[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 6.0f, f.c[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.02f, f.c[2]);
    const float target = f.eval(37.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 37.5f, f.inverse(target, 0, 100));
}

void test_quadratic_inverse_stays_on_increasing_branch() {
    // y = 100 − (x − 5)² on [0, 10] peaks at x = 5
    const float x[] = {0, 2, 4, 6, 8, 10};
    float y[6];
    for (int i = 0; i < 6; ++i) y[i] = 100.0f - (x[i] - 5) * (x[i] - 5);
    CurveFit f;
    TEST_ASSERT_TRUE(fitCurve(x, y, 6, FitType::QUADRATIC, f, nullptr));
    const float c = f.inverse(91.0f, 0, 10);   // 91 at x = 2 (and 8)
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 2.0f, c);
    TEST_ASSERT_FLOAT_WITHIN(1e-2, 5.0f, f.inverse(500.0f, 0, 10));   // saturates at the peak
}

void test_lookup_monotone_and_inverse() {
    const float x[] = {0, 10, 20, 30};
    const float y[] = {0, 100, 250, 300};
    CurveFit f;
    TEST_ASSERT_TRUE(fitCurve(x, y, 4, FitType::LOOKUP, f, nullptr));
    TEST_ASSERT_FALSE(f.nonMonotone);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 175.0f, f.eval(15));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 15.0f, f.inverse(175.0f, 0, 30));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 26.0f, f.inverse(280.0f, 0, 30));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 30.0f, f.inverse(1000.0f, 0, 30));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, f.r2);
}

void test_lookup_duplicates_and_non_monotone() {
    // Up/down pass duplicates; a dip at x = 20 must be repaired and flagged.
    const float x[] = {0, 10, 20, 30, 30, 20, 10, 0};
    const float y[] = {0, 100, 90, 300, 310, 94, 104, 2};
    CurveFit f;
    TEST_ASSERT_TRUE(fitCurve(x, y, 8, FitType::LOOKUP, f, nullptr));
    TEST_ASSERT_EQUAL(4, f.n);
    TEST_ASSERT_TRUE(f.nonMonotone);
    for (int i = 1; i < f.n; ++i) TEST_ASSERT_TRUE(f.ly[i] >= f.ly[i - 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 97.0f, f.ly[1]);   // (102 + 92) / 2 pooled
    TEST_ASSERT_TRUE(f.usableIn(0, 30));
}

void test_fit_errors() {
    const float x[] = {5, 5, 5};
    const float y[] = {1, 2, 3};
    CurveFit f;
    const char* err = nullptr;
    TEST_ASSERT_FALSE(fitCurve(x, y, 3, FitType::LINEAR, f, &err));
    TEST_ASSERT_FALSE(fitCurve(x, y, 1, FitType::LINEAR, f, &err));
    const float x2[] = {1, 2};
    TEST_ASSERT_FALSE(fitCurve(x2, y, 2, FitType::QUADRATIC, f, &err));
}

void test_negative_slope_not_usable() {
    const float x[] = {0, 1, 2};
    const float y[] = {10, 5, 0};
    CurveFit f;
    TEST_ASSERT_TRUE(fitCurve(x, y, 3, FitType::LINEAR, f, nullptr));
    TEST_ASSERT_FALSE(f.usableIn(0, 2));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_linear_exact);
    RUN_TEST(test_linear_noisy_known_data);
    RUN_TEST(test_quadratic_exact);
    RUN_TEST(test_quadratic_inverse_stays_on_increasing_branch);
    RUN_TEST(test_lookup_monotone_and_inverse);
    RUN_TEST(test_lookup_duplicates_and_non_monotone);
    RUN_TEST(test_fit_errors);
    RUN_TEST(test_negative_slope_not_usable);
    return UNITY_END();
}
