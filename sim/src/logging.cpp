#include "sixsim/sim/logging.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

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

}  // namespace sixsim::sim
