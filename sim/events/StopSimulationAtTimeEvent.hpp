#pragma once

#include "sixsim/math/comparison.hpp"
#include "sixsim/sim/simulation_event.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

namespace sixsim::sim {

struct StopSimulationAtTimeEventConfig {
  std::optional<double> stop_time_s{};
};

class StopSimulationAtTimeEvent final : public SimulationEvent {
 public:
  explicit StopSimulationAtTimeEvent(const StopSimulationAtTimeEventConfig& config)
      : config_(config) {}

  void verify_config() const override {
    if (!config_.stop_time_s.has_value()) {
      throw std::invalid_argument("StopSimulationAtTimeEvent.config.stop_time_s is required");
    }
    if (!std::isfinite(*config_.stop_time_s) || *config_.stop_time_s < 0.0) {
      throw std::invalid_argument(
          "StopSimulationAtTimeEvent.config.stop_time_s must be finite and nonnegative");
    }
  }

  bool stops_simulation() const override { return true; }

 protected:
  bool condition(const EventContext& context) const override {
    return math::compare_geq(context.time.simtime_s, config_.stop_time_s.value());
  }

  void action(EventContext& context) override {
    context.stop_simulation = flag();
  }

 private:
  StopSimulationAtTimeEventConfig config_{};
};

}  // namespace sixsim::sim
