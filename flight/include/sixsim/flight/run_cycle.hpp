#pragma once

namespace sixsim::flight {

template <typename Fcu>
void run_cycle(Fcu& fcu) {
  auto& timer = fcu.devices().sensors().timer();
  auto& serial = fcu.devices().serial();
  const double adjusted_time = timer.read() * timer.config().multiplier;

  serial.print("Timer reading: ");
  serial.print(adjusted_time);
  serial.println();
}

}  // namespace sixsim::flight
