#include "sim/models/gravity/constant_gravity.hpp"

namespace sixsim::sim {

ConstantGravity::ConstantGravity(double gravity_mps2)
    : gravity_mps2_(gravity_mps2) {}

GravityState ConstantGravity::evaluate(
    const SimTime& time,
    const math::Vector3& position_ned_m) const {
  (void)time;
  (void)position_ned_m;
  GravityState state{};
  state.acceleration_ned_mps2 = {0.0, 0.0, gravity_mps2_};
  return state;
}

}  // namespace sixsim::sim
