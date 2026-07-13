#pragma once

#include <cmath>

namespace sixsim::math {

constexpr double eps = 1.0e-9;

inline bool compare_eq(double a, double b, double tolerance = eps) {
  return std::abs(a - b) <= tolerance;
}

inline bool compare_neq(double a, double b, double tolerance = eps) {
  return !compare_eq(a, b, tolerance);
}

inline bool compare_gt(double a, double b, double tolerance = eps) {
  return a > b && compare_neq(a, b, tolerance);
}

inline bool compare_lt(double a, double b, double tolerance = eps) {
  return a < b && compare_neq(a, b, tolerance);
}

inline bool compare_geq(double a, double b, double tolerance = eps) {
  return a > b || compare_eq(a, b, tolerance);
}

inline bool compare_leq(double a, double b, double tolerance = eps) {
  return a < b || compare_eq(a, b, tolerance);
}

}  // namespace sixsim::math
