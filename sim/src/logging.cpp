#include "sixsim/sim/logging.hpp"

#include "sim/models/dynamics/rigid_body.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>

namespace sixsim::sim {

namespace {

std::filesystem::path stream_path(const std::filesystem::path& raw_directory,
                                  const char* stream) {
  return raw_directory / (std::string(stream) + ".csv");
}

void validate_stream_name(const char* stream) {
  if (stream == nullptr || stream[0] == '\0') {
    throw std::runtime_error("log stream name must not be empty");
  }
}

void validate_field(const LogField& field) {
  if (field.name == nullptr || field.name[0] == '\0') {
    throw std::runtime_error("log field name must not be empty");
  }
}

void write_header(const std::filesystem::path& path,
                  const std::vector<std::string>& field_names) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to open log stream: " + path.string());
  }

  output << "time_s";
  for (const std::string& field_name : field_names) {
    output << ',' << field_name;
  }
  output << '\n';
}

void write_row(const std::filesystem::path& path,
               const SimTime& time,
               std::span<const LogField> fields) {
  std::ofstream output(path, std::ios::app);
  if (!output) {
    throw std::runtime_error("failed to open log stream: " + path.string());
  }

  output << std::setprecision(17) << time.simtime_s;
  for (const LogField& field : fields) {
    output << ',' << std::setprecision(17) << field.value;
  }
  output << '\n';
}

void validate_field_names(const char* stream,
                          const std::vector<std::string>& expected,
                          std::span<const LogField> actual) {
  if (expected.size() != actual.size()) {
    throw std::runtime_error("log stream changed field count: " +
                             std::string(stream));
  }

  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (expected[i] != actual[i].name) {
      throw std::runtime_error("log stream changed field name: " +
                               std::string(stream));
    }
  }
}

}  // namespace

LogSink::LogSink(std::filesystem::path run_directory)
    : raw_directory_(std::move(run_directory) / "raw") {
  std::filesystem::create_directories(raw_directory_);
}

void LogSink::log_sample(const char* stream,
                         const SimTime& time,
                         std::span<const LogField> fields) {
  validate_stream_name(stream);
  if (fields.empty()) {
    throw std::runtime_error("log sample must contain at least one field");
  }

  for (const LogField& field : fields) {
    validate_field(field);
  }

  const std::filesystem::path path = stream_path(raw_directory_, stream);
  std::vector<std::string>& field_names = stream_fields_[std::string(stream)];
  if (field_names.empty()) {
    for (const LogField& field : fields) {
      field_names.emplace_back(field.name);
    }
    write_header(path, field_names);
  } else {
    validate_field_names(stream, field_names, fields);
  }

  write_row(path, time, fields);
}

void log_truth_sample(LogSink& log,
                      const SimTime& time,
                      const RigidBodyState& state) {
  const LogField fields[] = {
      {"position_ned_m.x", state.position_ned_m.x},
      {"position_ned_m.y", state.position_ned_m.y},
      {"position_ned_m.z", state.position_ned_m.z},
      {"velocity_body_mps.x", state.velocity_body_mps.x},
      {"velocity_body_mps.y", state.velocity_body_mps.y},
      {"velocity_body_mps.z", state.velocity_body_mps.z},
      {"q_body2ned.w", state.q_body2ned.w},
      {"q_body2ned.x", state.q_body2ned.x},
      {"q_body2ned.y", state.q_body2ned.y},
      {"q_body2ned.z", state.q_body2ned.z},
      {"omega_body_rps.x", state.omega_body_rps.x},
      {"omega_body_rps.y", state.omega_body_rps.y},
      {"omega_body_rps.z", state.omega_body_rps.z},
  };

  log.log_sample("truth", time, fields);
}

SimulationLogger::SimulationLogger(std::filesystem::path run_directory,
                                   double logging_rate_hz)
    : log_(std::move(run_directory)),
      logging_period_s_(1.0 / logging_rate_hz) {}

void SimulationLogger::log_truth(const SimTime& time,
                                 const RigidBodyState& state) {
  if (time.simtime_s < next_log_time_s_) {
    return;
  }

  log_truth_sample(log_, time, state);
  do {
    next_log_time_s_ += logging_period_s_;
  } while (next_log_time_s_ <= time.simtime_s);
}

}  // namespace sixsim::sim
