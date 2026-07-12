#pragma once

#include "sixsim/math/vector3.hpp"

namespace sixsim::sim {

struct ForceMoment {
  math::Vector3 force_body_n{};
  math::Vector3 moment_body_nm{};
};

}  // namespace sixsim::sim
