#pragma once

#include "sixsim/hal/altimeter.hpp"

#include <Adafruit_BMP5xx.h>
#include <Arduino.h>
#include <Wire.h>

namespace sixsim::hal {

class Physical_irec_2526_fcu_Altimeter final : public altimeter {
 public:
  void begin() {
    Wire.begin();
    Wire.setClock(100000UL);

    initialized_ = bmp_.begin(0x47, &Wire) || bmp_.begin(0x46, &Wire);
    if (!initialized_) {
      return;
    }

    bmp_.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp_.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    bmp_.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp_.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp_.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    bmp_.enablePressure(true);
    bmp_.configureInterrupt(BMP5XX_INTERRUPT_LATCHED,
                            BMP5XX_INTERRUPT_ACTIVE_HIGH,
                            BMP5XX_INTERRUPT_PUSH_PULL,
                            BMP5XX_INTERRUPT_DATA_READY, true);
  }

  AltimeterSample read() override {
    if (initialized_ && bmp_.dataReady() && bmp_.performReading()) {
      sample_.altitude_msl_m = bmp_.readAltitude(1013.25);
      sample_.measurement_time_s = static_cast<double>(millis()) / 1000.0;
      sample_.valid = true;
    }
    return sample_;
  }

 private:
  Adafruit_BMP5xx bmp_;
  AltimeterSample sample_{};
  bool initialized_{false};
};

}  // namespace sixsim::hal
