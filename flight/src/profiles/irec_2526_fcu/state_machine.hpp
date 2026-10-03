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
      {
        if (!standby_entered) {
          hal_adapter.enter();
          LaunchDetection.enter();
          DoubleAltitude.enter();
          TripleAltitude.enter();
          standby_entered = true;
        }

        hal_adapter.run(fcu, common);
        runLaunchDetection(common, LaunchDetection);
        runDoubleAltitude(common, DoubleAltitude);
        runTripleAltitude(common, TripleAltitude);

        if (common.enter_powered_ascent) {
          state = State::POWERED_ASCENT;
          transition_pending = true;
          common.enter_powered_ascent = false;
        }

        if (transition_pending) {
          LaunchDetection.exit();
          DoubleAltitude.exit();
          TripleAltitude.exit();
          standby_entered = false;
          transition_pending = false;
        }

        break;
      }
      case State::POWERED_ASCENT:
      {
        if (!powered_ascent_entered) {
          // do nothing
          powered_ascent_entered = true;
        }

        hal_adapter.run(fcu, common);
        break;
      }
      case State::COASTING_ASCENT:
      {
        if (!coasting_ascent_entered) {
          // do nothing
          coasting_ascent_entered = true;
        }

        hal_adapter.run(fcu, common);
        break;
      }
      case State::DROGUE_DESCENT:
      {
        if (!drogue_descent_entered) {
          // do nothing
          drogue_descent_entered = true;
        }

        hal_adapter.run(fcu, common);
        break;
      }
      case State::MAIN_DESCENT:
      {
        if (!main_descent_entered) {
          // do nothing
          main_descent_entered = true;
        }

        hal_adapter.run(fcu, common);
        break;
      }
      case State::LANDED:
      {
        if (!landed_entered) {
          // do nothing
          landed_entered = true;
        }

        hal_adapter.run(fcu, common);
        break;
      }
    }
  }

 private:
  using State = generated::State;

  CommonValues common{};
  HalAdapter hal_adapter;
  State state{State::Standby};
  bool standby_entered{false};
  bool transition_pending{false};
  bool powered_ascent_entered{false};
  bool coasting_ascent_entered{false};
  bool drogue_descent_entered{false};
  bool main_descent_entered{false};
  bool landed_entered{false};

  // Algorithms
  algorithms::LaunchDetection LaunchDetection;
  algorithms::DoubleAltitude DoubleAltitude;
  algorithms::TripleAltitude TripleAltitude;
};

}  // namespace sixsim::flight::irec_2526_fcu
