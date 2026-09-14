#pragma once

#include "sixsim/hal/altimeter.hpp"

namespace sixsim::hal {

class SitlAltimeter : public Altimeter {
 public:
  void update(const AltimeterSample& sample) {
    sample_ = sample;
  }

  AltimeterSample read() override {
    return sample_;
  }

 private:
  AltimeterSample sample_{};
};

}  // namespace sixsim::hal
