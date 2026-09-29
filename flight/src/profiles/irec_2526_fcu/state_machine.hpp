#pragma once

#include "common_values.hpp"
#include "hal_adapter.hpp"
#include "flight/build/generated/include/sixsim/flight/profiles/irec_2526_fcu/AlgorithmBindings.hpp"
#include "flight/build/generated/include/sixsim/flight/profiles/irec_2526_fcu/State.hpp"

namespace sixsim::flight::irec_2526_fcu {

class StateMachine {
 public:
  template <typename Fcu>
  void update(Fcu& fcu) {
    switch (state) {
      case State::Standby:
        if (!standby_entered) {
          hal_adapter.enter();
          DoubleAltitude.enter();
          TripleAltitude.enter();
          standby_entered = true;
        }

        hal_adapter.run(fcu, common);
        runDoubleAltitude(common, DoubleAltitude);
        runTripleAltitude(common, TripleAltitude);

        break;
    }
  }

 private:
  using State = generated::State;

  CommonValues common{};
  HalAdapter hal_adapter;
  State state{State::Standby};
  bool standby_entered{false};

  // Algorithms
  algorithms::DoubleAltitude DoubleAltitude;
  algorithms::TripleAltitude TripleAltitude;
};

}  // namespace sixsim::flight::irec_2526_fcu
