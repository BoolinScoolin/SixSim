#include "sim/models/propulsion/hardcoded_thrust_curve_propulsion.hpp"

#include <array>
#include <stdexcept>

namespace sixsim::sim {
namespace {

struct ThrustCurveSample {
  double time_s{};
  double thrust_n{};
  double mass_kg{};
};

constexpr std::array<ThrustCurveSample, 37> kThrustCurve{{
    {0.011, 1195.177, 4.751},
    {0.024, 2029.903, 4.74065329511549},
    {0.037, 2380.868, 4.72650265517784},
    {0.05, 2542.122, 4.71070871373162},
    {0.1, 2570.578, 4.64762190889266},
    {0.15, 2561.093, 4.58430101643373},
    {0.2, 2523.151, 4.5215653368551},
    {0.25, 2485.208, 4.45976602009557},
    {0.3, 2523.151, 4.39796670333604},
    {0.35, 2570.578, 4.33511398611704},
    {0.4, 2674.919, 4.27038856793832},
    {0.5, 2912.057, 4.13251055258363},
    {0.6, 3073.311, 3.9848008325119},
    {0.7, 3073.311, 3.83311161056151},
    {0.8, 3101.768, 3.68072011341199},
    {0.9, 3092.282, 3.52786044102254},
    {1.0, 3092.282, 3.37523486859229},
    {1.1, 2959.485, 3.22588652284157},
    {1.186, 2807.716, 3.10348642731455},
    {1.227, 2437.781, 3.05041158440799},
    {1.27, 2257.556, 3.00058588062798},
    {1.3, 2162.701, 2.96786032770717},
    {1.4, 1991.961, 2.86532962934198},
    {1.5, 1878.135, 2.76982158425424},
    {1.6, 1792.765, 2.67922939152502},
    {1.7, 1688.424, 2.59331897587359},
    {1.8, 1612.54, 2.51185623734074},
    {1.9, 1584.083, 2.43296847496676},
    {2.0, 1536.656, 2.3559534135525},
    {2.048, 1498.714, 2.31999743666908},
    {2.084, 1403.858, 2.29421026449539},
    {2.102, 1166.72, 2.28279143707751},
    {2.134, 796.784, 2.26728542969686},
    {2.186, 455.305, 2.25121761696399},
    {2.237, 237.138, 2.24250251599363},
    {2.3, 94.855, 2.23734087619966},
    {2.4, 0.0, 2.235},
}};

double interpolate_thrust(double time_s) {
  if (time_s < kThrustCurve.front().time_s ||
      time_s > kThrustCurve.back().time_s) {
    return 0.0;
  }

  for (std::size_t i = 1; i < kThrustCurve.size(); ++i) {
    const ThrustCurveSample& previous = kThrustCurve[i - 1];
    const ThrustCurveSample& next = kThrustCurve[i];
    if (time_s <= next.time_s) {
      const double segment_duration_s = next.time_s - previous.time_s;
      const double fraction = (time_s - previous.time_s) / segment_duration_s;
      return previous.thrust_n +
             fraction * (next.thrust_n - previous.thrust_n);
    }
  }

  return 0.0;
}

double interpolate_mass(double time_s) {
  if (time_s < kThrustCurve.front().time_s) {
    return kThrustCurve.front().mass_kg;
  }
  if (time_s > kThrustCurve.back().time_s) {
    return kThrustCurve.back().mass_kg;
  }

  for (std::size_t i = 1; i < kThrustCurve.size(); ++i) {
    const ThrustCurveSample& previous = kThrustCurve[i - 1];
    const ThrustCurveSample& next = kThrustCurve[i];
    if (time_s <= next.time_s) {
      const double segment_duration_s = next.time_s - previous.time_s;
      const double fraction = (time_s - previous.time_s) / segment_duration_s;
      return previous.mass_kg + fraction * (next.mass_kg - previous.mass_kg);
    }
  }

  return kThrustCurve.back().mass_kg;
}

}  // namespace

HardcodedThrustCurvePropulsion::HardcodedThrustCurvePropulsion(
    const math::Vector3& thrust_direction_body) {
  const double direction_norm = math::norm(thrust_direction_body);
  if (direction_norm == 0.0) {
    throw std::runtime_error("thrust direction must be nonzero");
  }

  thrust_direction_body_ = thrust_direction_body / direction_norm;
}

ForceMoment HardcodedThrustCurvePropulsion::evaluate(
    const SimTime& time,
    const RigidBodyState& state,
    VehicleContext& vehicle) const {
  (void)state;

  vehicle.mass_properties.mass_kg =
      vehicle.unloaded_mass_kg + interpolate_mass(time.simtime_s);
  ForceMoment force_moment{};
  force_moment.force_body_n =
      thrust_direction_body_ * interpolate_thrust(time.simtime_s);
  return force_moment;
}

}  // namespace sixsim::sim
