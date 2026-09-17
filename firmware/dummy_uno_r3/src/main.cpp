#include <Arduino.h>
#include "hal/physical/dummy_uno_r3/hal.hpp"
#include "dummy_uno_r3_config.hpp"
#include "sixsim/flight/run_cycle.hpp"

sixsim::hal::PhysicalDummyUnoR3Fcu fcu{
    sixsim::hal::generated::flight_timing};

void setup() {
    auto& devices = fcu.devices();
    devices.begin(sixsim::hal::generated::serial_baud_rate);
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
