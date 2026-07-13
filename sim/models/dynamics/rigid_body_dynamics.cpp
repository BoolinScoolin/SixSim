#include "sim/models/dynamics/rigid_body_dynamics.hpp"

namespace sixsim::sim {

ForceMoment gravity_force_moment_body(const GravityState& gravity,
                                      const MassProperties& mass_properties,
                                      const math::Quaternion& q_body2ned) {
  ForceMoment force_moment{};
  const math::Vector3 force_ned =
      gravity.acceleration_ned_mps2 * mass_properties.mass_kg;
  force_moment.force_body_n =
      math::rotate(math::conjugate(q_body2ned), force_ned);
  return force_moment;
}

}  // namespace sixsim::sim
