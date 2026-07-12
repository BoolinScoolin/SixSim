#include "constant_wind.hpp"

namespace sixsim::sim {

ConstantWind::ConstantWind(const math::Vector3& wind_ned_mps)
    : wind_ned_mps_(wind_ned_mps) {}

WindState ConstantWind::evaluate(
    const SimTime& time,
    const math::Vector3& position_ned_m) const {
  (void)time;
  (void)position_ned_m;
  WindState state{};
  state.wind_ned_mps = wind_ned_mps_;
  return state;
}

}  // namespace sixsim::sim
