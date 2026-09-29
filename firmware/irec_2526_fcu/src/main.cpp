#include <Arduino.h>
#include "hal/physical/irec_2526_fcu/hal.hpp"
#include "irec_2526_fcu_config.hpp"
#include "flight/src/profiles/irec_2526_fcu/flight_program.hpp"
#include "sixsim/flight/runtime.hpp"

#include <utility>

using FlightRuntime = sixsim::flight::FlightRuntime<
    sixsim::hal::irec_2526_fcu::PhysicalFcu,
    sixsim::flight::irec_2526_fcu::FlightProgram>;

FlightRuntime runtime{
    std::in_place, sixsim::hal::generated::flight_timing};

void setup() {
    auto& fcu = runtime.fcu();
    auto& devices = fcu.devices();
    sixsim::hal::generated::configure_devices(devices);
    devices.begin();
    if (fcu.check_cycle().due) {
        runtime.run_cycle();
    }
}

void loop() 
{
    const auto update = runtime.fcu().check_cycle();
    if (update.due) {
        runtime.run_cycle();
    }
}
