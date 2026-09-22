#include <csignal>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <termios.h>
#include <unistd.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>

#include <gtsam_points/config.hpp>
#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#endif

#include <aimcap/reader.hpp>
#include <aimcap/types.hpp>

namespace {

struct PlaybackFinishedException : public std::exception {
  const char* what() const noexcept override { return "PlaybackFinishedException"; }
};

std::atomic<bool> g_shutdown_requested{false};
aimcap::Reader* g_active_reader = nullptr;

void signal_handler(int sig) {
  g_shutdown_requested.store(true);
  if (g_active_reader) {
    g_active_reader->Stop();
  }
}

class KeyboardHandler {
public:
  KeyboardHandler() : paused_(false), active_(false) {
    if (!isatty(STDIN_FILENO)) return;
    tcgetattr(STDIN_FILENO, &orig_termios_);
    struct termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    active_ = true;
  }

  ~KeyboardHandler() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }
  }

  void update() {
    if (!active_) return;
    char c;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == ' ') {
        paused_ = !paused_;
        if (paused_) {
          spdlog::info("[keyboard] Playback PAUSED (press Space to resume)");
        } else {
          spdlog::info("[keyboard] Playback RESUMED");
        }
      } else if (c == 'q' || c == 'Q') {
        g_shutdown_requested.store(true);
      }
    }
  }

  bool is_paused() const { return paused_; }

private:
  bool paused_;
  bool active_;
  struct termios orig_termios_;
};

inline const aimcap::PointField* findField(const std::vector<aimcap::PointField>& fields, const std::string& name) {
  for (const auto& f : fields) {
    if (f.name == name) return &f;
  }
  return nullptr;
}

inline double readFieldAsDouble(const uint8_t* base, const aimcap::PointField& f) {
  const uint8_t* p = base + f.offset;
  switch (f.type) {
    case aimcap::PointFieldType::FLOAT32: {
      float v; std::memcpy(&v, p, sizeof(v)); return v;
    }
    case aimcap::PointFieldType::FLOAT64: {
      double v; std::memcpy(&v, p, sizeof(v)); return v;
    }
    case aimcap::PointFieldType::UINT16: {
      uint16_t v; std::memcpy(&v, p, sizeof(v)); return v;
    }
    case aimcap::PointFieldType::UINT32: {
      uint32_t v; std::memcpy(&v, p, sizeof(v)); return v;
    }
    case aimcap::PointFieldType::INT16: {
      int16_t v; std::memcpy(&v, p, sizeof(v)); return v;
    }
    case aimcap::PointFieldType::INT32: {
      int32_t v; std::memcpy(&v, p, sizeof(v)); return v;
    }
    case aimcap::PointFieldType::UINT8: {
      return *p;
    }
    case aimcap::PointFieldType::INT8: {
      return *reinterpret_cast<const int8_t*>(p);
    }
    default:
      return 0.0;
  }
}

void print_usage(const char* prog) {
  std::cout << "Usage: " << prog << " [options] <mcap_file>\n\n"
            << "Options:\n"
            << "  -i, --input <file>       Input MCAP recording\n"
            << "  -c, --config <dir>       Path to config folder (default: config)\n"
            << "  -o, --output <dir>       Output directory for results (default: ./glim_results)\n"
            << "  -d, --duration <sec>     Maximum duration in seconds to process (default: 0 = entire file)\n"
            << "  -s, --start <sec>        Offset in seconds from start of bag to begin processing\n"
            << "  -r, --rate <float>       Playback speed multiplier (e.g. 1.0 = real-time, 0 = unlimited max speed)\n"
            << "  --lidar <topic>          LiDAR topic name (default: /ouster/points)\n"
            << "  --imu <topic>            IMU topic name (default: /ouster/imu)\n"
            << "  --cpu                    Use CPU-only estimation and mapping (default: GPU)\n"
            << "  --headless               Run without visualizer GUI\n"
            << "  -h, --help               Display this help\n";
}

} // namespace

int main(int argc, char** argv) {
  std::string input_mcap;
  std::string config_path = "config";
  std::string output_dir = "./glim_results";
  std::string lidar_topic = "/ouster/points";
  std::string imu_topic = "/ouster/imu";
  double max_duration = 0.0;
  double start_offset = 0.0;
  double playback_rate = 0.0;
  bool headless = false;
  bool use_cpu = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return 0;
    } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
      input_mcap = argv[++i];
    } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      config_path = argv[++i];
    } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
      output_dir = argv[++i];
    } else if ((arg == "-d" || arg == "--duration") && i + 1 < argc) {
      max_duration = std::stod(argv[++i]);
    } else if ((arg == "-s" || arg == "--start") && i + 1 < argc) {
      start_offset = std::stod(argv[++i]);
    } else if ((arg == "-r" || arg == "--rate") && i + 1 < argc) {
      playback_rate = std::stod(argv[++i]);
    } else if (arg == "--lidar" && i + 1 < argc) {
      lidar_topic = argv[++i];
    } else if (arg == "--imu" && i + 1 < argc) {
      imu_topic = argv[++i];
    } else if (arg == "--headless") {
      headless = true;
    } else if (arg == "--cpu") {
      use_cpu = true;
    } else if (arg[0] != '-' && input_mcap.empty()) {
      input_mcap = arg;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      print_usage(argv[0]);
      return 1;
    }
  }

  if (input_mcap.empty()) {
    std::cerr << "Error: No input MCAP file specified.\n";
    print_usage(argv[0]);
    return 1;
  }

  // Setup signal handlers
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Setup logger
  auto console_logger = spdlog::stdout_color_mt("glim");
  spdlog::set_default_logger(console_logger);
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  spdlog::info("GLIM Standalone MCAP Runner starting... ({})", use_cpu ? "CPU mode" : "GPU mode");
  spdlog::info("Input MCAP: {}", input_mcap);
  spdlog::info("Config path: {}", config_path);
  spdlog::info("Output dir: {}", output_dir);

  // Initialize GLIM GlobalConfig
  auto global_config = glim::GlobalConfig::instance(config_path);
  if (use_cpu) {
    global_config->override_param("global", "config_odometry", std::string("config_odometry_cpu.json"));
    global_config->override_param("global", "config_sub_mapping", std::string("config_sub_mapping_cpu.json"));
    std::string current_global = global_config->param<std::string>("global", "config_global_mapping", "");
    if (current_global.find("pose_graph") == std::string::npos) {
      global_config->override_param("global", "config_global_mapping", std::string("config_global_mapping_cpu.json"));
    }
  }

  // Setup GPU linearization hook if available and in GPU mode
#ifdef GTSAM_POINTS_USE_CUDA
  if (!use_cpu) {
    gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
  }
#endif

  // Initialize TimeKeeper & Preprocessor
  auto time_keeper = std::make_unique<glim::TimeKeeper>();
  auto preprocessor = std::make_unique<glim::CloudPreprocessor>();

  // Load Odometry module
  glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
  std::string odom_so = config_odometry.param<std::string>("odometry_estimation", "so_name", use_cpu ? "libodometry_estimation_cpu.so" : "libodometry_estimation_gpu.so");
  spdlog::info("Loading odometry module: {}", odom_so);
  auto odom = glim::OdometryEstimationBase::load_module(odom_so);
  if (!odom) {
    spdlog::warn("Failed to load {}, falling back to CPU odometry", odom_so);
    odom = glim::OdometryEstimationBase::load_module("libodometry_estimation_cpu.so");
  }
  if (!odom) {
    spdlog::critical("Could not load any odometry estimation module!");
    return 1;
  }
  auto odometry_estimation = std::make_shared<glim::AsyncOdometryEstimation>(odom, odom->requires_imu());

  // Load SubMapping module
  std::shared_ptr<glim::AsyncSubMapping> sub_mapping;
  std::string sub_so = glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping"))
                           .param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
  if (!sub_so.empty()) {
    spdlog::info("Loading sub-mapping module: {}", sub_so);
    auto sub = glim::SubMappingBase::load_module(sub_so);
    if (sub) {
      sub_mapping = std::make_shared<glim::AsyncSubMapping>(sub);
    }
  }

  // Load GlobalMapping module
  std::shared_ptr<glim::AsyncGlobalMapping> global_mapping;
  std::string global_so = glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping"))
                              .param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
  if (!global_so.empty()) {
    spdlog::info("Loading global mapping module: {}", global_so);
    auto global = glim::GlobalMappingBase::load_module(global_so);
    if (global) {
      global_mapping = std::make_shared<glim::AsyncGlobalMapping>(global);
    }
  }

  // Load Viewer if requested
  std::shared_ptr<glim::ExtensionModule> viewer_module;
  if (!headless) {
    const char* disp = std::getenv("DISPLAY");
    if (!disp || std::strlen(disp) == 0) {
      spdlog::warn("DISPLAY environment variable not set. Falling back to --headless mode.");
      headless = true;
    }
  }

  if (!headless) {
    spdlog::info("Initializing 3D Visualizer...");
    viewer_module = glim::ExtensionModule::load_module("libinteractive_viewer.so");
    if (!viewer_module) {
      viewer_module = glim::ExtensionModule::load_module("libstandard_viewer.so");
    }
    if (viewer_module) {
      spdlog::info("Visualizer loaded successfully");
    } else {
      spdlog::warn("Visualizer library not found, running headless");
    }
  }

  // Open MCAP reader
  spdlog::info("Scanning MCAP reader on: {}", input_mcap);
  std::unique_ptr<aimcap::Reader> reader;
  try {
    reader = std::make_unique<aimcap::Reader>(input_mcap, /*add_column_timestamps=*/true);
    g_active_reader = reader.get();
  } catch (const std::exception& e) {
    spdlog::critical("Failed to open MCAP file: {}", e.what());
    return 1;
  }

  std::ofstream tum_file;
  std::filesystem::create_directories(output_dir);
  std::string tum_path = output_dir + "/trajectory_tum.txt";
  tum_file.open(tum_path);
  if (tum_file.is_open()) {
    tum_file << std::fixed << std::setprecision(6);
    spdlog::info("Writing trajectory to {}", tum_path);
  }

  KeyboardHandler keyboard;
  double bag_start_time = -1.0;
  size_t lidar_count = 0;
  size_t imu_count = 0;
  auto wall_start = std::chrono::steady_clock::now();
  auto last_stat_time = wall_start;
  double last_stat_bag_time = 0.0;

  // IMU Callback
  reader->OnImu(imu_topic, [&](const std::string&, const aimcap::DecodedImu& imu) {
    if (g_shutdown_requested.load()) {
      reader->Stop();
      return;
    }
    const double stamp = imu.time_ns * 1e-9;
    if (bag_start_time < 0.0) bag_start_time = stamp;

    const double rel_bag_time = stamp - bag_start_time;
    if (start_offset > 0.0 && rel_bag_time < start_offset) return;
    if (max_duration > 0.0 && (rel_bag_time - start_offset) > max_duration) {
      g_shutdown_requested.store(true);
      reader->Stop();
      return;
    }

    Eigen::Vector3d linear_acc(imu.linear_acceleration_mps2[0],
                               imu.linear_acceleration_mps2[1],
                               imu.linear_acceleration_mps2[2]);
    Eigen::Vector3d angular_vel(imu.angular_velocity_radps[0],
                                imu.angular_velocity_radps[1],
                                imu.angular_velocity_radps[2]);

    if (!time_keeper->validate_imu_stamp(stamp)) return;

    odometry_estimation->insert_imu(stamp, linear_acc, angular_vel);
    if (sub_mapping) sub_mapping->insert_imu(stamp, linear_acc, angular_vel);
    if (global_mapping) global_mapping->insert_imu(stamp, linear_acc, angular_vel);
    imu_count++;
  });

  // Point Cloud Callback
  reader->OnPointCloud(lidar_topic, [&](const std::string&, const aimcap::DecodedPointCloud& cloud) {
    if (g_shutdown_requested.load()) {
      reader->Stop();
      return;
    }
    const double stamp = cloud.stamp_sec + cloud.stamp_nsec * 1e-9;
    if (bag_start_time < 0.0) bag_start_time = stamp;

    const double rel_bag_time = stamp - bag_start_time;
    if (start_offset > 0.0 && rel_bag_time < start_offset) return;
    if (max_duration > 0.0 && (rel_bag_time - start_offset) > max_duration) {
      g_shutdown_requested.store(true);
      reader->Stop();
      return;
    }

    keyboard.update();
    while (keyboard.is_paused() && !g_shutdown_requested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      keyboard.update();
    }

    // Workload throttling across all stages to prevent unbounded GPU memory growth
    while ((odometry_estimation->workload() > 5 ||
           (sub_mapping && sub_mapping->workload() > 5) ||
           (global_mapping && global_mapping->workload() > 2)) &&
           !g_shutdown_requested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // Playback rate throttling if requested
    if (playback_rate > 0.0) {
      auto wall_now = std::chrono::steady_clock::now();
      double wall_elapsed = std::chrono::duration<double>(wall_now - wall_start).count();
      double target_wall_time = (rel_bag_time - start_offset) / playback_rate;
      if (target_wall_time > wall_elapsed) {
        std::this_thread::sleep_for(std::chrono::duration<double>(target_wall_time - wall_elapsed));
      }
    }

    auto raw_points = std::make_shared<glim::RawPoints>();
    raw_points->stamp = stamp;

    const auto* fx = findField(cloud.fields, "x");
    const auto* fy = findField(cloud.fields, "y");
    const auto* fz = findField(cloud.fields, "z");
    const auto* fi = findField(cloud.fields, "intensity");
    if (!fi) fi = findField(cloud.fields, "reflectivity");
    const auto* ft = findField(cloud.fields, "t");

    if (!fx || !fy || !fz) {
      spdlog::warn("Point cloud missing x, y, or z field!");
      return;
    }

    const size_t num_points = cloud.width * cloud.height;
    raw_points->points.reserve(num_points);
    raw_points->times.reserve(num_points);
    raw_points->intensities.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* p = cloud.data.data() + i * cloud.point_step;
      float x = static_cast<float>(readFieldAsDouble(p, *fx));
      float y = static_cast<float>(readFieldAsDouble(p, *fy));
      float z = static_cast<float>(readFieldAsDouble(p, *fz));

      if (x == 0.0f && y == 0.0f && z == 0.0f) continue;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      double intensity = fi ? readFieldAsDouble(p, *fi) : 0.0;
      double t_rel = ft ? (readFieldAsDouble(p, *ft) * 1e-9) : 0.0;

      raw_points->points.emplace_back(x, y, z, 1.0);
      raw_points->intensities.push_back(intensity);
      raw_points->times.push_back(t_rel);
    }

    if (!time_keeper->process(raw_points)) return;
    auto preprocessed = preprocessor->preprocess(raw_points);
    odometry_estimation->insert_frame(preprocessed);
    lidar_count++;

    // Transfer results to sub_mapping and global_mapping
    std::vector<glim::EstimationFrame::ConstPtr> est_frames, marg_frames;
    odometry_estimation->get_results(est_frames, marg_frames);

    if (sub_mapping) {
      for (const auto& frame : marg_frames) {
        sub_mapping->insert_frame(frame);

        // Record TUM trajectory
        if (tum_file.is_open()) {
          const Eigen::Vector3d t = frame->T_world_lidar.translation();
          const Eigen::Quaterniond q(frame->T_world_lidar.linear());
          tum_file << frame->stamp << " "
                   << t.x() << " " << t.y() << " " << t.z() << " "
                   << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
          tum_file.flush();
        }
      }

      auto submaps = sub_mapping->get_results();
      if (global_mapping) {
        for (const auto& submap : submaps) {
          global_mapping->insert_submap(submap);
        }
      }
    }

    // Periodic progress reporting
    auto wall_now = std::chrono::steady_clock::now();
    double dt_stat = std::chrono::duration<double>(wall_now - last_stat_time).count();
    if (dt_stat >= 5.0) {
      double bag_dt = rel_bag_time - last_stat_bag_time;
      double speed = dt_stat > 0 ? (bag_dt / dt_stat) : 0.0;
      int sub_wl = sub_mapping ? sub_mapping->workload() : 0;
      int glob_wl = global_mapping ? global_mapping->workload() : 0;
      spdlog::info("[progress] bag_time: {:.1f}s | scans: {} | imu: {} | speed: {:.2f}x | queue: odom={} sub={} glob={}",
                   rel_bag_time, lidar_count, imu_count, speed, odometry_estimation->workload(), sub_wl, glob_wl);
      last_stat_time = wall_now;
      last_stat_bag_time = rel_bag_time;
    }
  });

  spdlog::info("Starting bag replay...");
  try {
    reader->Run();
  } catch (const PlaybackFinishedException&) {
    spdlog::info("Playback reached target limit or was paused/stopped by user.");
  } catch (const std::exception& e) {
    spdlog::warn("Reader stopped: {}", e.what());
  }
  g_active_reader = nullptr;

  spdlog::info("Replay finished or interrupted. Finalizing estimation and mapping...");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> est_frames, marg_frames;
    odometry_estimation->get_results(est_frames, marg_frames);
    for (const auto& frame : marg_frames) {
      sub_mapping->insert_frame(frame);
      if (tum_file.is_open()) {
        const Eigen::Vector3d t = frame->T_world_lidar.translation();
        const Eigen::Quaterniond q(frame->T_world_lidar.linear());
        tum_file << frame->stamp << " "
                 << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
      }
    }
    sub_mapping->join();

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      global_mapping->join();

      spdlog::info("Saving global map to {}", output_dir);
      global_mapping->save(output_dir);
    }
  }

  if (tum_file.is_open()) {
    tum_file.close();
    spdlog::info("Trajectory written to {}", tum_path);
  }

  spdlog::info("GLIM mapping completed successfully. Total scans: {}, IMU samples: {}", lidar_count, imu_count);
  return 0;
}
