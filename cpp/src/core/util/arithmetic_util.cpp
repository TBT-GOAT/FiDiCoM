

// include std libraries
#include <cmath>
#include <float.h>

// include header
#include "core/util/arithmetic_util.h"


int cmp_vals(const double& x, const double& y, double rel_tolerance, double abs_epsilon) {
    if (x == DBL_MAX && y == DBL_MAX) return 0;
    if (x == DBL_MAX) return 1; 
    if (y == DBL_MAX) return -1;

    double diff = std::fabs(x - y);
    double max_xy = std::max(std::fabs(x), std::fabs(y));
    if (diff <= std::max(rel_tolerance * max_xy, abs_epsilon)) {
        // 2つの値の比がrel_tolerance倍以下，
        // or
        // 2つの値の絶対誤差がabs_epsilon以下
        return 0;
    }
    return (x > y) ? 1 : -1;
}
int cmp_vals(const float& x, const float& y, float rel_tolerance, float abs_epsilon) {
    if (x == FLT_MAX && y == FLT_MAX) return 0;
    if (x == FLT_MAX) return 1; 
    if (y == FLT_MAX) return -1;

    float diff = std::fabs(x - y);
    float max_xy = std::max(std::fabs(x), std::fabs(y));
    if (diff <= std::max(rel_tolerance * max_xy, abs_epsilon)) {
        // 2つの値の比がrel_tolerance倍以下，
        // or
        // 2つの値の絶対誤差がabs_epsilon以下
        return 0;
    }
    return (x > y) ? 1 : -1;
}
int cmp_vals(const double& x, const float& y, float rel_tolerance, float abs_epsilon) {
    if (x == DBL_MAX && y == FLT_MAX) return 0;
    if (x == DBL_MAX) return 1; 
    if (y == FLT_MAX) return -1;

    float diff = std::fabs(x - y);
    float max_xy = std::max((float)std::fabs(x), std::fabs(y));
    if (diff <= std::max(rel_tolerance * max_xy, abs_epsilon)) {
        // 2つの値の比がrel_tolerance倍以下，
        // or
        // 2つの値の絶対誤差がabs_epsilon以下
        return 0;
    }
    return (x > y) ? 1 : -1;
}
int cmp_vals(const float& x, const double& y, float rel_tolerance, float abs_epsilon) {
    if (x == FLT_MAX && y == DBL_MAX) return 0;
    if (x == FLT_MAX) return 1; 
    if (y == DBL_MAX) return -1;

    float diff = std::fabs(x - y);
    float max_xy = std::max(std::fabs(x), (float)std::fabs(y));
    if (diff <= std::max(rel_tolerance * max_xy, abs_epsilon)) {
        // 2つの値の比がrel_tolerance倍以下，
        // or
        // 2つの値の絶対誤差がabs_epsilon以下
        return 0;
    }
    return (x > y) ? 1 : -1;
}