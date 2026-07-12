#pragma once

#include <cmath>

namespace sixsim::math {

struct Vector3 {
  double x{};
  double y{};
  double z{};

  constexpr Vector3 operator+(const Vector3& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }

  constexpr Vector3 operator-(const Vector3& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }

  constexpr Vector3 operator*(double scalar) const {
    return {x * scalar, y * scalar, z * scalar};
  }

  constexpr Vector3 operator/(double scalar) const {
    return {x / scalar, y / scalar, z / scalar};
  }

  constexpr Vector3& operator+=(const Vector3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  constexpr Vector3& operator-=(const Vector3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  constexpr Vector3& operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  constexpr Vector3& operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }
};

constexpr Vector3 operator*(double scalar, const Vector3& vector) {
  return vector * scalar;
}

constexpr double dot(const Vector3& a, const Vector3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vector3 cross(const Vector3& a, const Vector3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

constexpr double norm_squared(const Vector3& vector) {
  return dot(vector, vector);
}

inline double norm(const Vector3& vector) {
  return std::sqrt(norm_squared(vector));
}

}  // namespace sixsim::math
