#include <iostream>
#include <string>
#include <glim/viewer/offline_viewer.hpp>
#include <glim/util/config.hpp>
#include <spdlog/spdlog.h>

int main(int argc, char** argv) {
  std::string map_path = "";
  std::string config_path = "config";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: glim_offline_viewer [options] [map_directory]\n\n"
                << "Options:\n"
                << "  -c, --config <dir>   Path to config directory (default: config)\n"
                << "  -h, --help           Show this help message\n";
      return 0;
    } else if (arg[0] != '-') {
      map_path = arg;
    }
  }

  glim::GlobalConfig::instance(config_path);

  spdlog::info("Launching GLIM Offline Viewer...");
  if (!map_path.empty()) {
    spdlog::info("Loading map from: {}", map_path);
  } else {
    spdlog::info("No initial map directory provided. Use File -> Open New Map in the GUI.");
  }

  glim::OfflineViewer viewer(map_path);
  viewer.wait();

  return 0;
}
