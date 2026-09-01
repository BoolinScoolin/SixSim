#include "sixsim/sim/run_simulation.hpp"

#include <filesystem>
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
  std::filesystem::path output_directory;
  if (argc == 3 && std::string_view(argv[1]) == "--output" &&
      argv[2][0] != '\0') {
    output_directory = argv[2];
  } else if (argc != 1) {
    std::cerr << "usage: " << argv[0] << " [--output <run-directory>]\n";
    return 1;
  }

  sixsim::sim::run_simulation(output_directory);
  return 0;
}
