#pragma once

#include <filesystem>
#include <vector>

namespace sixsim::sim {

class RunArtifacts {
 public:
  RunArtifacts(const std::filesystem::path& output_directory,
               const std::filesystem::path& default_run_directory,
               const std::filesystem::path& source_scenario_path,
               const std::vector<std::filesystem::path>& fcu_config_paths);

  const std::filesystem::path& run_directory() const;
  void save_manifest() const;

 private:
  std::filesystem::path run_directory_;
  std::filesystem::path source_scenario_path_;
  std::vector<std::filesystem::path> fcu_config_paths_;
};

}  // namespace sixsim::sim
