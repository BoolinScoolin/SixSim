#pragma once

#include "sixsim/math.hpp"

#include <cstdint>

namespace sixsim::flight {

inline void read_i32(std::int32_t& algorithm_value,
                     const std::int32_t& common_value) {
  algorithm_value = common_value;
}

inline void write_i32(std::int32_t& common_value,
                      const std::int32_t& algorithm_value) {
  common_value = algorithm_value;
}

inline void read_f64(double& algorithm_value, const double& common_value) {
  algorithm_value = common_value;
}

inline void write_f64(double& common_value, const double& algorithm_value) {
  common_value = algorithm_value;
}

inline void read_bool(bool& algorithm_value, const bool& common_value) {
  algorithm_value = common_value;
}

inline void write_bool(bool& common_value, const bool& algorithm_value) {
  common_value = algorithm_value;
}

inline void read_v3(Vector3& algorithm_value, const Vector3& common_value) {
  algorithm_value = common_value;
}

inline void write_v3(Vector3& common_value, const Vector3& algorithm_value) {
  common_value = algorithm_value;
}

inline void read_quat(Quaternion& algorithm_value,
                      const Quaternion& common_value) {
  algorithm_value = common_value;
}

inline void write_quat(Quaternion& common_value,
                       const Quaternion& algorithm_value) {
  common_value = algorithm_value;
}

}  // namespace sixsim::flight
