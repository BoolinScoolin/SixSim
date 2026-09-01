#pragma once

#include "sixsim/sim/sim_general.hpp"

#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace sixsim::sim {

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

}  // namespace sixsim::sim
