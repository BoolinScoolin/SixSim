#pragma once

namespace sixsim::flight {

template <typename Fcu>
void run_cycle(Fcu& fcu) {
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
  serial.println();
}

}  // namespace sixsim::flight
