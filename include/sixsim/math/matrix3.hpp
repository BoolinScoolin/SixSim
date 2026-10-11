#pragma once

#include <array>
#include <cstddef>

#include "sixsim/math/vector3.hpp"

namespace sixsim::math {

struct Matrix3 {
  std::array<double, 9> values{};

  static constexpr Matrix3 identity();

  constexpr double& operator()(std::size_t row, std::size_t column) {
    return values[row * 3 + column];
  }

  constexpr double operator()(std::size_t row, std::size_t column) const {
    return values[row * 3 + column];
  }

  constexpr Matrix3 transpose() const {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        result(row, column) = (*this)(column, row);
      }
    }
    return result;
  }

  constexpr Matrix3 operator+(const Matrix3& other) const {
    Matrix3 result{};
    for (std::size_t i = 0; i < values.size(); ++i) {
      result.values[i] = values[i] + other.values[i];
    }
    return result;
  }

  constexpr Matrix3 operator-(const Matrix3& other) const {
    Matrix3 result{};
    for (std::size_t i = 0; i < values.size(); ++i) {
      result.values[i] = values[i] - other.values[i];
    }
    return result;
  }

  constexpr Matrix3& operator+=(const Matrix3& other) {
    for (std::size_t i = 0; i < values.size(); ++i) {
      values[i] += other.values[i];
    }
    return *this;
  }

  constexpr Matrix3& operator-=(const Matrix3& other) {
    for (std::size_t i = 0; i < values.size(); ++i) {
      values[i] -= other.values[i];
    }
    return *this;
  }

  constexpr Vector3 operator*(const Vector3& vector) const {
    return {
        (*this)(0, 0) * vector.x + (*this)(0, 1) * vector.y +
            (*this)(0, 2) * vector.z,
        (*this)(1, 0) * vector.x + (*this)(1, 1) * vector.y +
            (*this)(1, 2) * vector.z,
        (*this)(2, 0) * vector.x + (*this)(2, 1) * vector.y +
            (*this)(2, 2) * vector.z,
    };
  }

  constexpr Matrix3 operator*(const Matrix3& other) const {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        for (std::size_t inner = 0; inner < 3; ++inner) {
          result(row, column) += (*this)(row, inner) * other(inner, column);
        }
      }
    }
    return result;
  }

  constexpr Matrix3 operator*(double scalar) const {
    Matrix3 result{};
    for (std::size_t i = 0; i < values.size(); ++i) {
      result.values[i] = values[i] * scalar;
    }
    return result;
  }
};

constexpr Matrix3 Matrix3::identity() {
  return {{{1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0}}};
}

constexpr Matrix3 operator*(double scalar, const Matrix3& matrix) {
  return matrix * scalar;
}

}  // namespace sixsim::math
