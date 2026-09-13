#include "sixsim/sim/run_artifacts.hpp"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace sixsim::sim {

RunArtifacts::RunArtifacts(
    const std::filesystem::path& output_directory,
    const std::filesystem::path& default_run_directory,
    const std::filesystem::path& source_scenario_path)
    : run_directory_(std::filesystem::absolute(
                         output_directory.empty() ? default_run_directory
                                                  : output_directory)
                         .lexically_normal()),
      source_scenario_path_(source_scenario_path) {
  const std::filesystem::path current_directory =
      std::filesystem::current_path().lexically_normal();
  if (run_directory_ == run_directory_.root_path() ||
      run_directory_ == current_directory) {
    throw std::runtime_error("refusing to replace unsafe run directory: " +
                             run_directory_.string());
  }

  const std::filesystem::path run_marker = run_directory_ / ".sixsim-run";
  if (std::filesystem::exists(run_directory_)) {
    std::ifstream marker_input(run_marker);
    std::string marker_value;
    std::getline(marker_input, marker_value);
    if (!marker_input || marker_value != "SixSim run directory" ||
        marker_input.peek() != std::char_traits<char>::eof()) {
      throw std::runtime_error(
          "refusing to replace unmarked run directory: " +
          run_directory_.string());
    }
  }

  std::filesystem::remove_all(run_directory_);
  const std::filesystem::path config_directory = run_directory_ / "configs";
  std::filesystem::create_directories(config_directory);

  std::ofstream marker_output(run_marker);
  marker_output << "SixSim run directory\n";
  if (!marker_output) {
    throw std::runtime_error("failed to write run directory marker: " +
                             run_marker.string());
  }

  std::filesystem::copy_file(
      source_scenario_path_,
      config_directory / "scenario.yaml",
      std::filesystem::copy_options::overwrite_existing);
}

const std::filesystem::path& RunArtifacts::run_directory() const {
  return run_directory_;
}

void RunArtifacts::save_manifest() const {
  std::ofstream manifest(run_directory_ / "manifest.yaml");
  if (!manifest) {
    throw std::runtime_error("failed to open run manifest");
  }
  manifest << "scenario:\n"
           << "  original: " << std::quoted(source_scenario_path_.string())
           << '\n'
           << "  copied: \"configs/scenario.yaml\"\n"
           << "raw:\n"
           << "  truth: \"raw/truth.csv\"\n";
  if (!manifest) {
    throw std::runtime_error("failed to write run manifest");
  }
}

}  // namespace sixsim::sim
