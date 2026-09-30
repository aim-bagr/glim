#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/preprocess/callbacks.hpp>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <glim/util/config.hpp>
#include <glim/util/convert_to_string.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/task_arena.h>
#include <tbb/parallel_for.h>
#endif

namespace glim {

namespace {

inline std::uint64_t splitmix64(std::uint64_t& state) {
  std::uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

/**
 * @brief Random grid sampling whose output does not depend on the number of threads or thread scheduling.
 * Same semantics as gtsam_points::randomgrid_sampling (about `sampling_rate * N` points, evenly spread over occupied voxels),
 * but points are sorted by (voxel, index) and each voxel is sampled with an RNG seeded from (per-frame seed, voxel key).
 */
gtsam_points::PointCloudCPU::Ptr randomgrid_sampling_deterministic(
  const gtsam_points::PointCloud::ConstPtr& frame,
  const double voxel_resolution,
  const double sampling_rate,
  std::mt19937& mt,
  const int num_threads) {
  if (sampling_rate >= 0.99) {
    return gtsam_points::PointCloudCPU::clone(*frame);
  }

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size = 21;
  constexpr std::int64_t coord_bit_mask = (1 << 21) - 1;
  constexpr int coord_offset = 1 << (coord_bit_size - 1);
  const double inv_resolution = 1.0 / voxel_resolution;

  const std::int64_t num_points = frame->size();
  std::vector<std::pair<std::uint64_t, std::int64_t>> coord_pt(num_points);

#pragma omp parallel for num_threads(num_threads) schedule(static)
  for (std::int64_t i = 0; i < num_points; i++) {
    std::uint64_t key = invalid_coord;
    if (frame->points[i].array().isFinite().all()) {
      const Eigen::Array4i coord = gtsam_points::fast_floor(frame->points[i] * inv_resolution) + coord_offset;
      if (!(coord < 0).any() && !(coord > coord_bit_mask).any()) {
        key = (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << 0) |   //
              (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << 21) |  //
              (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << 42);
      }
    }
    coord_pt[i] = {key, i};
  }

  // (key, index) is a total order, so the result is independent of the sort algorithm
  std::sort(coord_pt.begin(), coord_pt.end());

  size_t num_voxels = 0;
  for (size_t i = 0; i < coord_pt.size() && coord_pt[i].first != invalid_coord; i++) {
    if (i == 0 || coord_pt[i - 1].first != coord_pt[i].first) {
      num_voxels++;
    }
  }
  if (num_voxels == 0) {
    return gtsam_points::PointCloudCPU::clone(*frame);
  }

  const size_t points_per_voxel = std::ceil((sampling_rate * num_points) / num_voxels);
  const size_t max_num_points = num_points * sampling_rate * 1.2;
  const std::uint64_t frame_seed = (static_cast<std::uint64_t>(mt()) << 32) | mt();

  std::vector<int> indices;
  indices.reserve(static_cast<size_t>(num_points * sampling_rate * 1.5));
  std::vector<int> voxel_indices;

  for (size_t begin = 0; begin < coord_pt.size() && coord_pt[begin].first != invalid_coord;) {
    size_t end = begin;
    while (end < coord_pt.size() && coord_pt[end].first == coord_pt[begin].first) {
      end++;
    }

    voxel_indices.clear();
    for (size_t i = begin; i < end; i++) {
      voxel_indices.push_back(coord_pt[i].second);
    }

    if (voxel_indices.size() <= points_per_voxel) {
      indices.insert(indices.end(), voxel_indices.begin(), voxel_indices.end());
    } else {
      // Partial Fisher-Yates shuffle
      std::uint64_t state = frame_seed ^ (coord_pt[begin].first * 0x2545f4914f6cdd1dULL);
      for (size_t k = 0; k < points_per_voxel; k++) {
        const size_t j = k + splitmix64(state) % (voxel_indices.size() - k);
        std::swap(voxel_indices[k], voxel_indices[j]);
        indices.push_back(voxel_indices[k]);
      }
    }
    begin = end;
  }

  if (indices.size() > max_num_points) {
    std::vector<int> sub_indices(max_num_points);
    std::sample(indices.begin(), indices.end(), sub_indices.begin(), max_num_points, mt);
    indices = std::move(sub_indices);
  }

  std::sort(indices.begin(), indices.end());
  return gtsam_points::sample(frame, indices);
}

}  // namespace

CloudPreprocessorParams::CloudPreprocessorParams() {
  Config config(GlobalConfig::get_config_path("config_preprocess"));
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));

  global_shutter = sensor_config.param<bool>("sensors", "global_shutter_lidar", false);

  distance_near_thresh = config.param<double>("preprocess", "distance_near_thresh", 1.0);
  distance_far_thresh = config.param<double>("preprocess", "distance_far_thresh", 100.0);
  use_random_grid_downsampling = config.param<bool>("preprocess", "use_random_grid_downsampling", false);
  downsample_resolution = config.param<double>("preprocess", "downsample_resolution", 0.15);
  downsample_target = config.param<int>("preprocess", "random_downsample_target", 0);
  downsample_rate = config.param<double>("preprocess", "random_downsample_rate", 0.3);
  enable_outlier_removal = config.param<bool>("preprocess", "enable_outlier_removal", false);
  outlier_removal_k = config.param<int>("preprocess", "outlier_removal_k", 10);
  outlier_std_mul_factor = config.param<double>("preprocess", "outlier_std_mul_factor", 2.0);

  enable_cropbox_filter = config.param<bool>("preprocess", "enable_cropbox_filter", false);
  crop_bbox_frame = "lidar";
  crop_bbox_min.setZero();
  crop_bbox_max.setZero();

  if (enable_cropbox_filter) {
    Eigen::Isometry3d T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
    T_imu_lidar = T_lidar_imu.inverse();

    crop_bbox_frame = config.param<std::string>("preprocess", "crop_bbox_frame", "lidar");
    crop_bbox_min = config.param<Eigen::Vector3d>("preprocess", "crop_bbox_min", Eigen::Vector3d(0.0, 0.0, 0.0));
    crop_bbox_max = config.param<Eigen::Vector3d>("preprocess", "crop_bbox_max", Eigen::Vector3d(0.0, 0.0, 0.0));

    if (crop_bbox_frame != "lidar" && crop_bbox_frame != "imu") {
      throw std::runtime_error(fmt::format("Unsupported crop bbox frame: {}", crop_bbox_frame));
    } else if ((crop_bbox_min.array() > crop_bbox_max.array()).any()) {
      throw std::runtime_error(fmt::format("Misconfigured bbox: min={}, max={}", convert_to_string(crop_bbox_min), convert_to_string(crop_bbox_max)));
    }
  }

  k_correspondences = config.param<int>("preprocess", "k_correspondences", 8);

  num_threads = config.param<int>("preprocess", "num_threads", 2);
}

CloudPreprocessorParams::~CloudPreprocessorParams() {}

CloudPreprocessor::CloudPreprocessor(const CloudPreprocessorParams& params) : params(params) {
#ifdef GTSAM_POINTS_USE_TBB
  if (gtsam_points::is_tbb_default()) {
    tbb_task_arena.reset(new tbb::task_arena(params.num_threads));
  }
#endif
}

CloudPreprocessor::~CloudPreprocessor() {}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess(const RawPoints::ConstPtr& raw_points) {
  PreprocessCallbacks::on_raw_points_received(raw_points);
  if (gtsam_points::is_omp_default() || params.num_threads == 1 || !tbb_task_arena) {
    return preprocess_impl(raw_points);
  }

  PreprocessedFrame::Ptr preprocessed;
#ifdef GTSAM_POINTS_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] { preprocessed = preprocess_impl(raw_points); });
#else
  std::cerr << "error : TBB is not enabled" << std::endl;
  abort();
#endif
  return preprocessed;
}

PreprocessedFrame::Ptr CloudPreprocessor::preprocess_impl(const RawPoints::ConstPtr& raw_points) {
  spdlog::trace("preprocessing input: {} points", raw_points->size());

  gtsam_points::PointCloudCPU::Ptr frame = std::make_shared<gtsam_points::PointCloudCPU>();
  frame->add_points(raw_points->points);
  frame->add_times(raw_points->times);
  if (raw_points->intensities.size()) {
    frame->add_intensities(raw_points->intensities);
  }
  PreprocessCallbacks::on_preprocessing_begin(frame);

  // Downsampling
  if (params.use_random_grid_downsampling) {
    const double rate = params.downsample_target > 0 ? static_cast<double>(params.downsample_target) / frame->size() : params.downsample_rate;
    frame = randomgrid_sampling_deterministic(frame, params.downsample_resolution, rate, mt, params.num_threads);
  } else {
    frame = gtsam_points::voxelgrid_sampling(frame, params.downsample_resolution, params.num_threads);
  }
  PreprocessCallbacks::on_downsampling_finished(frame);

  if (frame->size() < 100) {
    spdlog::warn("too few points in the downsampled cloud ({} points)", frame->size());
  }

  // Distance filter
  std::vector<int> indices;
  indices.reserve(frame->size());
  double squared_distance_near_thresh = params.distance_near_thresh * params.distance_near_thresh;
  double squared_distance_far_thresh = params.distance_far_thresh * params.distance_far_thresh;

  for (int i = 0; i < frame->size(); i++) {
    const bool is_finite = frame->points[i].allFinite();
    const double squared_dist = (Eigen::Vector4d() << frame->points[i].head<3>(), 0.0).finished().squaredNorm();
    if (squared_dist > squared_distance_near_thresh && squared_dist < squared_distance_far_thresh && is_finite) {
      indices.push_back(i);
    }
  }

  if (indices.size() < 100) {
    spdlog::warn("too few points in the filtered cloud ({} points)", indices.size());
  }

  // Sort by time
  std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) { return frame->times[lhs] < frame->times[rhs]; });
  frame = gtsam_points::sample(frame, indices);

  if (params.global_shutter) {
    std::fill(frame->times, frame->times + frame->size(), 0.0);
  }

  // Cropbox filter
  if (params.enable_cropbox_filter) {
    if (params.crop_bbox_frame == "lidar") {
      auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) {
        return (p_lidar.array() >= params.crop_bbox_min.array()).all() && (p_lidar.array() <= params.crop_bbox_max.array()).all();
      };

      frame = gtsam_points::filter(frame, [&](const auto& pt) { return !is_inside_bbox(pt.template head<3>()); });

    } else if (params.crop_bbox_frame == "imu") {
      auto is_inside_bbox = [&](const Eigen::Vector3d& p_lidar) {
        const auto p_imu = params.T_imu_lidar * p_lidar;
        return (p_imu.array() >= params.crop_bbox_min.array()).all() && (p_imu.array() <= params.crop_bbox_max.array()).all();
      };

      frame = gtsam_points::filter(frame, [&](const auto& pt) { return !is_inside_bbox(pt.template head<3>()); });

    } else {
      throw std::runtime_error(fmt::format("Unsupported crop bbox frame: {}", params.crop_bbox_frame));
    }
  }

  // Outlier removal
  if (params.enable_outlier_removal) {
    frame = gtsam_points::remove_outliers(frame, params.outlier_removal_k, params.outlier_std_mul_factor, params.num_threads);
  }

  PreprocessCallbacks::on_filtering_finished(frame);

  // Create a preprocessed frame
  PreprocessedFrame::Ptr preprocessed(new PreprocessedFrame);
  preprocessed->stamp = raw_points->stamp;
  preprocessed->scan_end_time = frame->size() ? raw_points->stamp + frame->times[frame->size() - 1] : raw_points->stamp;

  preprocessed->times.assign(frame->times, frame->times + frame->size());
  preprocessed->points.assign(frame->points, frame->points + frame->size());
  if (frame->intensities) {
    preprocessed->intensities.assign(frame->intensities, frame->intensities + frame->size());
  }

  preprocessed->k_neighbors = params.k_correspondences;
  preprocessed->neighbors = find_neighbors(frame->points, frame->size(), params.k_correspondences);

  spdlog::trace("preprocessed: {} -> {} points", raw_points->size(), preprocessed->size());

  return preprocessed;
}

std::vector<int> CloudPreprocessor::find_neighbors(const Eigen::Vector4d* points, const int num_points, const int k) const {
  gtsam_points::KdTree tree(points, num_points);

  std::vector<int> neighbors(num_points * k);

  const auto perpoint_task = [&](int i) {
    std::vector<size_t> k_indices(k, i);
    std::vector<double> k_sq_dists(k);
    size_t num_found = tree.knn_search(points[i].data(), k, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.begin() + num_found, neighbors.begin() + i * k);
  };

  if (gtsam_points::is_omp_default()) {
#pragma omp parallel for num_threads(params.num_threads) schedule(guided, 8)
    for (int i = 0; i < num_points; i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, num_points, 8), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error : TBB is not enabled" << std::endl;
    abort();
#endif
  }

  return neighbors;
}

}  // namespace glim
