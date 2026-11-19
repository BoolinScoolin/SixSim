#include "sim/models/atmosphere/isa1976_atmosphere.hpp"

#include <cmath>
#include <stdexcept>

namespace sixsim::sim {
namespace {

constexpr double earth_radius_m = 6356766.0;
constexpr double gravity_m_per_s2 = 9.80665;
constexpr double gas_constant_j_per_kg_k = 8314.32 / 28.9644;
constexpr double specific_heat_ratio = 1.4;
constexpr double layer_altitudes_m[] = {
    0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0, 84852.0};
constexpr double lapse_rates_k_per_m[] = {
    -0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002};
constexpr double minimum_geopotential_altitude_m = -5000.0;
constexpr double maximum_geopotential_altitude_m = 84852.0;
constexpr double minimum_geometric_altitude_m =
    earth_radius_m * minimum_geopotential_altitude_m /
    (earth_radius_m - minimum_geopotential_altitude_m);
constexpr double maximum_geometric_altitude_m =
    earth_radius_m * maximum_geopotential_altitude_m /
    (earth_radius_m - maximum_geopotential_altitude_m);

}  // namespace

Isa1976Atmosphere::Isa1976Atmosphere(double origin_altitude_msl_m)
    : origin_altitude_msl_m_(origin_altitude_msl_m) {
  if (!std::isfinite(origin_altitude_msl_m_)) {
    throw std::invalid_argument("ISA 1976 origin altitude must be finite");
  }
}

AtmosphereState Isa1976Atmosphere::evaluate(
    const SimTime& time,
    const math::Vector3& position_ned_m) const {
  (void)time;
  const double geometric_altitude_m =
      origin_altitude_msl_m_ - position_ned_m.z;
  if (!std::isfinite(geometric_altitude_m)) {
    throw std::invalid_argument("ISA 1976 altitude must be finite");
  }
  if (geometric_altitude_m < minimum_geometric_altitude_m ||
      geometric_altitude_m > maximum_geometric_altitude_m) {
    throw std::out_of_range("Altitude outside ISA 1976 supported range");
  }
  const double geopotential_altitude_m =
      earth_radius_m * (geometric_altitude_m /
                        (earth_radius_m + geometric_altitude_m));

  double temperature_k = 288.15;
  double pressure_pa = 101325.0;
  for (int layer = 0; layer < 7; ++layer) {
    const double top_altitude_m =
        std::fmin(geopotential_altitude_m, layer_altitudes_m[layer + 1]);
    const double height_m = top_altitude_m - layer_altitudes_m[layer];
    const double lapse_rate = lapse_rates_k_per_m[layer];
    const double top_temperature_k = temperature_k + lapse_rate * height_m;

    if (lapse_rate == 0.0) {
      pressure_pa *= std::exp(-gravity_m_per_s2 * height_m /
                              (gas_constant_j_per_kg_k * temperature_k));
    } else {
      pressure_pa *= std::pow(temperature_k / top_temperature_k,
                              gravity_m_per_s2 /
                                  (gas_constant_j_per_kg_k * lapse_rate));
    }
    temperature_k = top_temperature_k;
    if (geopotential_altitude_m <= layer_altitudes_m[layer + 1]) {
      break;
    }
  }

  AtmosphereState state{};
  state.temperature_k = temperature_k;
  state.pressure_pa = pressure_pa;
  state.density_kg_per_m3 =
      pressure_pa / (gas_constant_j_per_kg_k * temperature_k);
  state.speed_of_sound_m_per_s =
      std::sqrt(specific_heat_ratio * gas_constant_j_per_kg_k * temperature_k);
  return state;
}

}  // namespace sixsim::sim
