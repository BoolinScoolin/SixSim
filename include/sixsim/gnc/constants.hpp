#pragma once

namespace sixsim {

inline constexpr double SIXSIM_PI = 3.141592653589793238462643383279502884;
inline constexpr double SIXSIM_EPS = 1e-12;

inline constexpr double SIXSIM_DEG_TO_RAD = SIXSIM_PI / 180.0;
inline constexpr double SIXSIM_RAD_TO_DEG = 180.0 / SIXSIM_PI;

inline constexpr double SIXSIM_MEAN_EARTH_RADIUS = 6371000.0;
inline constexpr double SIXSIM_EQUATORIAL_EARTH_RADIUS = 6378137.0;
inline constexpr double SIXSIM_GRAVITATIONAL_PARAMETER_EARTH = 3.986004418e14;

}  // namespace sixsim
