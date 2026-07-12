#pragma once

#include <cmath>

#include "sixsim/math/vector3.hpp"

namespace sixsim::math {

struct Quaternion {
  double w{};
  double x{};
  double y{};
  double z{};

  constexpr Quaternion operator*(const Quaternion& other) const {
    return {
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w,
    };
  }

  constexpr Quaternion& operator*=(const Quaternion& other) {
    *this = *this * other;
    return *this;
  }
};

constexpr Quaternion conjugate(const Quaternion& quaternion) {
  return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

constexpr double norm_squared(const Quaternion& quaternion) {
  return quaternion.w * quaternion.w + quaternion.x * quaternion.x +
         quaternion.y * quaternion.y + quaternion.z * quaternion.z;
}

inline double norm(const Quaternion& quaternion) {
  return std::sqrt(norm_squared(quaternion));
}

inline Quaternion normalized(const Quaternion& quaternion) {
  const double magnitude = norm(quaternion);
  return {
      quaternion.w / magnitude,
      quaternion.x / magnitude,
      quaternion.y / magnitude,
      quaternion.z / magnitude,
  };
}

constexpr Quaternion positive_scalar(const Quaternion& quaternion) {
  if (quaternion.w >= 0.0) {
    return quaternion;
  }

  return {-quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

constexpr Vector3 rotate(const Quaternion& quaternion, const Vector3& vector) {
  const Quaternion pure_vector{0.0, vector.x, vector.y, vector.z};
  const Quaternion rotated = quaternion * pure_vector * conjugate(quaternion);
  return {rotated.x, rotated.y, rotated.z};
}

}  // namespace sixsim::math
