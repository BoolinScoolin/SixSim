#pragma once

#include "sixsim/sim/sim_general.hpp"

#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace sixsim::sim {

struct RigidBodyState;

struct LogField {
  const char* name{};
  double value{};
};

class LogSink {
 public:
  explicit LogSink(std::filesystem::path run_directory);

  void log_sample(const char* stream,
                  const SimTime& time,
                  std::span<const LogField> fields);

 private:
  std::filesystem::path raw_directory_;
  std::map<std::string, std::vector<std::string>> stream_fields_;
};

void log_truth_sample(LogSink& log,
                      const SimTime& time,
                      const RigidBodyState& state);

class SimulationLogger {
 public:
  SimulationLogger(std::filesystem::path run_directory,
                   double logging_rate_hz);

  void log_truth(const SimTime& time, const RigidBodyState& state);

 private:
  LogSink log_;
  double logging_period_s_{};
  double next_log_time_s_{};
};

}  // namespace sixsim::sim
