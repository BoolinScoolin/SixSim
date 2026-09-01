#pragma once

namespace sixsim {

inline constexpr double PI = 3.141592653589793238462643383279502884;
inline constexpr double EPS = 1e-12;

inline constexpr double DEG_TO_RAD = PI / 180.0;
inline constexpr double RAD_TO_DEG = 180.0 / PI;

inline constexpr double MEAN_EARTH_RADIUS = 6'371'000.0;
inline constexpr double EQUATORIAL_EARTH_RADIUS = 6'378'137.0;
inline constexpr double GRAVITATIONAL_PARAMETER_EARTH = 3.986004418e14;

}  // namespace sixsim
