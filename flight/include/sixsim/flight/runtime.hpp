#pragma once

#include <utility>

namespace sixsim::flight {

template <typename Fcu, typename FlightProgram>
class FlightRuntime {
 public:
  template <typename... FcuArgs>
  explicit FlightRuntime(std::in_place_t, FcuArgs&&... fcu_args)
      : fcu_(std::forward<FcuArgs>(fcu_args)...) {}

  FlightRuntime(const FlightRuntime&) = delete;
  FlightRuntime& operator=(const FlightRuntime&) = delete;
  FlightRuntime(FlightRuntime&&) = delete;
  FlightRuntime& operator=(FlightRuntime&&) = delete;

  Fcu& fcu() { return fcu_; }
  const Fcu& fcu() const { return fcu_; }

  void run_cycle() { program_.update(fcu_); }

 private:
  Fcu fcu_;
  FlightProgram program_{};
};

}  // namespace sixsim::flight
