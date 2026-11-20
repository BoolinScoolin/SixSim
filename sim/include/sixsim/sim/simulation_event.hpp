#pragma once

#include "sixsim/sim/scenario.hpp"
#include "sixsim/sim/sim_general.hpp"

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sixsim::sim {

struct SimulationEventFlag {
  bool triggered{};
  double trigger_time_s{};
};

struct EventContext {
  const SimTime& time;
  MassProperties& mass_properties;
  double& unloaded_mass_kg;
  SimulationEventFlag& stop_simulation;
};

class SimulationEvent {
 public:
  virtual ~SimulationEvent() = default;

  virtual void verify_config() const = 0;

  virtual bool stops_simulation() const { return false; }

  void update(EventContext& context) {
    if (flag_.triggered || !condition(context)) {
      return;
    }

    flag_ = SimulationEventFlag{true, context.time.simtime_s};
    action(context);
  }

  const SimulationEventFlag& flag() const {
    return flag_;
  }

 protected:
  virtual bool condition(const EventContext& context) const = 0;
  virtual void action(EventContext& context) = 0;

 private:
  SimulationEventFlag flag_{};
};

class SimulationEvents {
 public:
  void add(std::unique_ptr<SimulationEvent> event) {
    events_.push_back(std::move(event));
  }

  void validate() const {
    bool has_stop_event = false;
    for (const auto& event : events_) {
      event->verify_config();
      has_stop_event = has_stop_event || event->stops_simulation();
    }
    if (!has_stop_event) {
      throw std::runtime_error("scenario requires at least one stop-capable event");
    }
  }

  void update(EventContext& context) {
    for (const auto& event : events_) {
      event->update(context);
    }
  }

 private:
  std::vector<std::unique_ptr<SimulationEvent>> events_;
};

template <typename VehicleType>
inline void evaluate_simulation_events(
    const SimTime& time,
    std::vector<VehicleType>& vehicles,
    SimulationEvents& events,
    SimulationEventFlag& stop_simulation) {
  VehicleType& vehicle = vehicles.front();
  EventContext context{
      time,
      vehicle.mass_properties,
      vehicle.unloaded_mass_kg,
      stop_simulation,
  };
  events.update(context);
}

}  // namespace sixsim::sim
