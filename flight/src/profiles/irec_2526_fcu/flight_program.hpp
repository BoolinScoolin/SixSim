#pragma once

#include "state_machine.hpp"
#include "sixsim/flight/runtime.hpp"

namespace sixsim::flight::irec_2526_fcu {

class FlightProgram {
 public:
  template <typename Fcu>
  void update(Fcu& fcu) {
    state_machine.update(fcu);
  }

 private:
  StateMachine state_machine;
};

template <typename Fcu>
using FlightRuntime = sixsim::flight::FlightRuntime<Fcu, FlightProgram>;

}  // namespace sixsim::flight::irec_2526_fcu
