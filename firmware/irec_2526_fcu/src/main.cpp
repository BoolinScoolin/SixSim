#include <Arduino.h>
#include "hal/physical/irec_2526_fcu/hal.hpp"
#include "irec_2526_fcu_config.hpp"
#include "sixsim/flight/run_cycle.hpp"

sixsim::hal::irec_2526_fcu::PhysicalFcu fcu{
    sixsim::hal::generated::flight_timing};

void setup() {
    auto& devices = fcu.devices();
    sixsim::hal::generated::configure_devices(devices);
    devices.begin();
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
