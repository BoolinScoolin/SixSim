#pragma once

namespace sixsim::flight {

template <typename Fcu>
void run_cycle(Fcu& fcu) {
  auto& timer = fcu.devices().sensors().timer();
  auto& serial = fcu.devices().serial();

  serial.print("Timer reading: ");
  serial.print(timer.read());
  serial.println();
}

}  // namespace sixsim::flight
