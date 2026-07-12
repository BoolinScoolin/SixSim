#pragma once

#include "sixsim/math/vector3.hpp"
#include "sixsim/sim/sim_time.hpp"

namespace sixsim::sim {

struct AtmosphereState {
  double density_kg_per_m3{};
  double pressure_pa{};
  double temperature_k{};
  double speed_of_sound_m_per_s{};
};

class AtmosphereModel {
 public:
  virtual ~AtmosphereModel() = default;

  virtual AtmosphereState evaluate(const SimTime& time,
                                   const math::Vector3& position_ned_m) const = 0;
};

}  // namespace sixsim::sim
