#pragma once

#include "sixsim/math/vector3.hpp"

namespace sixsim::sim {

struct ForceMoment {
  math::Vector3 force_body_n{};
  math::Vector3 moment_body_nm{};
};

inline ForceMoment combine_force_moment(const ForceMoment& a,
                                        const ForceMoment& b) {
  ForceMoment combined{};
  combined.force_body_n = a.force_body_n + b.force_body_n;
  combined.moment_body_nm = a.moment_body_nm + b.moment_body_nm;
  return combined;
}

inline ForceMoment combine_force_moment(const ForceMoment& gravity,
                                        const ForceMoment& aerodynamics,
                                        const ForceMoment& propulsion) {
  return combine_force_moment(
      combine_force_moment(gravity, aerodynamics), propulsion);
}

}  // namespace sixsim::sim
