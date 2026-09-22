#pragma once

namespace sixsim::hal {

struct AltimeterSample {
  double altitude_msl_m{};
  double measurement_time_s{};  // Measurement time since runtime start.
  bool valid{false};
};

class altimeter {
 public:
  virtual ~altimeter() = default;

  // Return the latest sample without waiting for a new measurement.
  // Until a measurement is available, return a sample with valid == false.
  virtual AltimeterSample read() = 0;
};

}  // namespace sixsim::hal
