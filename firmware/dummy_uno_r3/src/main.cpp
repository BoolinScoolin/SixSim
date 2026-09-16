#include <Arduino.h>
#include "hal/fcu/dummy_uno_r3/fcu.hpp"
#include "dummy_uno_r3_config.hpp"
#include "sixsim/flight/run_cycle.hpp"

sixsim::hal::DummyUnoR3Fcu fcu{sixsim::hal::generated::flight_timing};

void setup() {
    auto& hardware = fcu.hardware();
    hardware.begin();
    if (fcu.check_cycle().due) {
        sixsim::flight::run_cycle(fcu);
    }
}

void loop() 
{
    const auto update = fcu.check_cycle();
    if (update.due) {
        sixsim::flight::run_cycle(fcu);
    }
}
