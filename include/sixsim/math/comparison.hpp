#pragma once

#include <cmath>

#include "sixsim/gnc/constants.hpp"

namespace sixsim::math {

inline bool compare_eq(double a, double b, double tolerance = SIXSIM_EPS) {
  return std::abs(a - b) <= tolerance;
}

inline bool compare_neq(double a, double b, double tolerance = SIXSIM_EPS) {
  return !compare_eq(a, b, tolerance);
}

inline bool compare_gt(double a, double b, double tolerance = SIXSIM_EPS) {
  return a > b && compare_neq(a, b, tolerance);
}

inline bool compare_lt(double a, double b, double tolerance = SIXSIM_EPS) {
  return a < b && compare_neq(a, b, tolerance);
}

inline bool compare_geq(double a, double b, double tolerance = SIXSIM_EPS) {
  return a > b || compare_eq(a, b, tolerance);
}

inline bool compare_leq(double a, double b, double tolerance = SIXSIM_EPS) {
  return a < b || compare_eq(a, b, tolerance);
}

}  // namespace sixsim::math
