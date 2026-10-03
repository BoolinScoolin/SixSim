#pragma once

#include "sixsim/flight/algorithm_io.hpp"

#include "common_values.hpp"

namespace sixsim::flight::irec_2526_fcu {

class HalAdapter {
 public:
  struct Outputs {
    double altitude_msl_m{};
    bool altitude_valid{false};
  };

  // Outputs
  Outputs outputs;

  void enter() {}

  template <typename Fcu>
  void run(Fcu& fcu, CommonValues& common) {

    auto& timer = fcu.devices().sensors().timer();
    const auto altimeter_sample = fcu.devices().sensors().altimeter().read();
    auto& serial = fcu.devices().serial();
    const double adjusted_time = timer.read() * timer.config().multiplier;

    serial.print("Timer reading: ");
    serial.print(adjusted_time);
    serial.print("   ");

    serial.print("Altimeter altitude MSL (m): ");
    serial.print(altimeter_sample.altitude_msl_m);
    serial.print(" measured at (s): ");
    serial.print(altimeter_sample.measurement_time_s);
    serial.print("   ");

    serial.print("Tripled Altimeter Readings: ");
    serial.print(common.tripled_altitude_msl_m_f64);

    serial.println();

    outputs.altitude_msl_m = altimeter_sample.altitude_msl_m;
    outputs.altitude_valid = altimeter_sample.valid;
    write_f64(common.altitude_msl_m_f64, outputs.altitude_msl_m);
    write_bool(common.altitude_valid_bool, outputs.altitude_valid);

  }

  void exit() {}
};

}  // namespace sixsim::flight::irec_2526_fcu
