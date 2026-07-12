#pragma once

#include "sixsim/math/vector3.hpp"

namespace sixsim::sim {

struct MassProperties {
  double mass_kg{};
  math::Vector3 inertia_body_kgm2{};
};

}  // namespace sixsim::sim
