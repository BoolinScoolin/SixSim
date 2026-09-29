#pragma once

#include "state_machine.hpp"

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

}  // namespace sixsim::flight::irec_2526_fcu
