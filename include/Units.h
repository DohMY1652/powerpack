#ifndef UNITS_H
#define UNITS_H

#include <cstddef>

static constexpr long double mL_per_m3 = 0.000001L;

inline long double operator"" _m3(long double val) { return val * mL_per_m3; }

#endif  // UNITS_H
