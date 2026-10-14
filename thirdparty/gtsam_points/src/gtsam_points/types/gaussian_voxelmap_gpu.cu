// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>

#include <thrust/pair.h>
#include <thrust/tuple.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/distance.h>

#include <set>
#include <vector>
#include <chrono>
#include <fstream>
#include <device_atomic_functions.h>
#include <gtsam_points/cuda/check_error.cuh>
#include <gtsam_points/cuda/kernels/vector3_hash.cuh>
#include <gtsam_points/cuda/cuda_malloc_async.hpp>
#include <gtsam_points/types/gaussian_voxel_data.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <thrust/iterator/constant_iterator.h>

namespace gtsam_points {

struct Vector3iCoordLess {
  __host__ __device__ bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
    if (a.x() != b.x()) return a.x() < b.x();
    if (a.y() != b.y()) return a.y() < b.y();
    return a.z() < b.z();
  }
};

struct Vector3iCoordEqual {
  __host__ __device__ bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
  }
};

// point coord -> voxel coord conversion
struct voxel_coord_kernel {
  voxel_coord_kernel(const VoxelMapInfo* info) : voxelmap_info_ptr(info) {}

  __host__ __device__ Eigen::Vector3i operator()(const Eigen::Vector3f& x) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);
    return calc_voxel_coord(x, info.voxel_resolution);
  }

  const thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
};

// assign voxel indices to buckets
struct voxel_bucket_assignment_kernel {
  voxel_bucket_assignment_kernel(
    const VoxelMapInfo* voxelmap_info,
    const Eigen::Vector3i* point_coords,
    thrust::pair<int, int>* index_buckets,
    int* voxels_failures)
  : voxelmap_info_ptr(voxelmap_info),
    point_coords_ptr(point_coords),
    index_buckets_ptr(index_buckets),
    voxels_failures_ptr(voxels_failures) {}

  __device__ void operator()(int point_index) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);
    const Eigen::Vector3i* coords = thrust::raw_pointer_cast(point_coords_ptr);
    uint64_t hash = vector3i_hash(coords[point_index]);

    for (int i = 0; i < info.max_bucket_scan_count; i++) {
      uint64_t bucket_index = (hash + i) % info.num_buckets;
      thrust::pair<int, int>* index_bucket = thrust::raw_pointer_cast(index_buckets_ptr) + bucket_index;

      int old = atomicCAS(&index_bucket->first, -1, point_index);
      if (old < 0) {
        index_bucket->second = atomicAdd(thrust::raw_pointer_cast(voxels_failures_ptr), 1);
        return;
      }

      if (equal(coords[point_index], coords[old])) {
        return;
      }
    }
    atomicAdd(thrust::raw_pointer_cast(voxels_failures_ptr) + 1, 1);
  }

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const Eigen::Vector3i> point_coords_ptr;
  thrust::device_ptr<thrust::pair<int, int>> index_buckets_ptr;
  thrust::device_ptr<int> voxels_failures_ptr;
};

// pair<point index, bucket index>  to pair<voxel coord, bucket index>
struct voxel_coord_select_kernel {
  voxel_coord_select_kernel(const Eigen::Vector3i* point_coords) : point_coords_ptr(point_coords) {}

  __device__ VoxelBucket operator()(const thrust::pair<int, int>& index_bucket) const {
    if (index_bucket.first < 0) {
      return {Eigen::Vector3i(0, 0, 0), -1};
    }

    return {thrust::raw_pointer_cast(point_coords_ptr)[index_bucket.first], index_bucket.second};
  }

  thrust::device_ptr<const Eigen::Vector3i> point_coords_ptr;
};

// accumulate points and covs
struct accumulate_points_kernel {
  accumulate_points_kernel(
    const VoxelMapInfo* voxelmap_info_ptr,
    const VoxelBucket* buckets,
    int* num_points,
    uint64_t* voxel_means_fixed,
    uint64_t* voxel_covs_fixed,
    float* voxel_intensities)
  : voxelmap_info_ptr(voxelmap_info_ptr),
    buckets_ptr(buckets),
    num_points_ptr(num_points),
    voxel_means_fixed_ptr(voxel_means_fixed),
    voxel_covs_fixed_ptr(voxel_covs_fixed),
    voxel_intensities_ptr(voxel_intensities) {}

  __device__ void operator()(const thrust::tuple<Eigen::Vector3f, Eigen::Matrix3f, float>& input) const {
    const auto& info = *thrust::raw_pointer_cast(voxelmap_info_ptr);

    const auto& mean = thrust::get<0>(input);
    const auto& cov = thrust::get<1>(input);
    const float intensity = static_cast<float>(thrust::get<2>(input));

    const Eigen::Vector3i coord = calc_voxel_coord(mean, info.voxel_resolution);
    uint64_t hash = vector3i_hash(coord);

    for (int i = 0; i < info.max_bucket_scan_count; i++) {
      uint64_t bucket_index = (hash + i) % info.num_buckets;
      const VoxelBucket& bucket = thrust::raw_pointer_cast(buckets_ptr)[bucket_index];

      if (bucket.second < 0) {
        break;
      }

      if (equal(bucket.first, coord)) {
        int& num_points = thrust::raw_pointer_cast(num_points_ptr)[bucket.second];
        float& voxel_intensity = thrust::raw_pointer_cast(voxel_intensities_ptr)[bucket.second];
        atomicAdd(&num_points, 1);

        constexpr double FIXED_SCALE = 4294967296.0;  // 2^32
        uint64_t* means_fixed = thrust::raw_pointer_cast(voxel_means_fixed_ptr) + bucket.second * 3;
        for (int j = 0; j < 3; j++) {
#ifdef __CUDA_ARCH__
          long long val = __double2ll_rn(static_cast<double>(mean[j]) * FIXED_SCALE);
#else
          long long val = std::llround(static_cast<double>(mean[j]) * FIXED_SCALE);
#endif
          atomicAdd(reinterpret_cast<unsigned long long*>(means_fixed + j), static_cast<unsigned long long>(val));
        }

        uint64_t* covs_fixed = thrust::raw_pointer_cast(voxel_covs_fixed_ptr) + bucket.second * 9;
        for (int j = 0; j < 9; j++) {
#ifdef __CUDA_ARCH__
          long long val = __double2ll_rn(static_cast<double>(cov.data()[j]) * FIXED_SCALE);
#else
          long long val = std::llround(static_cast<double>(cov.data()[j]) * FIXED_SCALE);
#endif
          atomicAdd(reinterpret_cast<unsigned long long*>(covs_fixed + j), static_cast<unsigned long long>(val));
        }

        unsigned int* as_uint = reinterpret_cast<unsigned int*>(&voxel_intensity);
        atomicMax(as_uint, __float_as_uint(intensity));  // Max intensity value
        // atomicAdd(thrust::raw_pointer_cast(voxel_intensities_ptr) + bucket.second, intensity); // Add intensity value for average intensity
      }
    }
  }

  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const VoxelBucket> buckets_ptr;

  thrust::device_ptr<int> num_points_ptr;
  thrust::device_ptr<uint64_t> voxel_means_fixed_ptr;
  thrust::device_ptr<uint64_t> voxel_covs_fixed_ptr;
  thrust::device_ptr<float> voxel_intensities_ptr;
};

struct finalize_voxels_kernel {
  finalize_voxels_kernel(
    const int* num_points,
    const uint64_t* voxel_means_fixed,
    const uint64_t* voxel_covs_fixed,
    Eigen::Vector3f* voxel_means,
    Eigen::Matrix3f* voxel_covs,
    float* voxel_intensities)
  : num_points_ptr(num_points),
    voxel_means_fixed_ptr(voxel_means_fixed),
    voxel_covs_fixed_ptr(voxel_covs_fixed),
    voxel_means_ptr(voxel_means),
    voxel_covs_ptr(voxel_covs),
    voxel_intensities_ptr(voxel_intensities) {}

  __host__ __device__ void operator()(int i) const {
    int num_points = thrust::raw_pointer_cast(num_points_ptr)[i];
    if (num_points == 0) {
      return;
    }
    auto& voxel_mean = thrust::raw_pointer_cast(voxel_means_ptr)[i];
    auto& voxel_covs = thrust::raw_pointer_cast(voxel_covs_ptr)[i];
    // auto& voxel_ints = thrust::raw_pointer_cast(voxel_intensities_ptr)[i];

    constexpr double INV_FIXED_SCALE = 1.0 / 4294967296.0;
    double inv_n = INV_FIXED_SCALE / static_cast<double>(num_points);

    const uint64_t* means_fixed = thrust::raw_pointer_cast(voxel_means_fixed_ptr) + i * 3;
    for (int j = 0; j < 3; j++) {
      int64_t val = static_cast<int64_t>(means_fixed[j]);
      voxel_mean[j] = static_cast<float>(static_cast<double>(val) * inv_n);
    }

    const uint64_t* covs_fixed = thrust::raw_pointer_cast(voxel_covs_fixed_ptr) + i * 9;
    for (int j = 0; j < 9; j++) {
      int64_t val = static_cast<int64_t>(covs_fixed[j]);
      voxel_covs.data()[j] = static_cast<float>(static_cast<double>(val) * inv_n);
    }
    // voxel_ints /= num_points;   //uncomment this line if you want to average the intensity values
  }

  thrust::device_ptr<const int> num_points_ptr;
  thrust::device_ptr<const uint64_t> voxel_means_fixed_ptr;
  thrust::device_ptr<const uint64_t> voxel_covs_fixed_ptr;
  thrust::device_ptr<Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<Eigen::Matrix3f> voxel_covs_ptr;
  thrust::device_ptr<float> voxel_intensities_ptr;
};

GaussianVoxelMapGPU::GaussianVoxelMapGPU(
  float resolution,
  int init_num_buckets,
  int max_bucket_scan_count,
  double target_points_drop_rate,
  CUstream_st* stream)
: stream(stream),
  init_num_buckets(init_num_buckets),
  target_points_drop_rate(target_points_drop_rate) {
  voxelmap_info.num_voxels = 0;
  voxelmap_info.num_buckets = init_num_buckets;
  voxelmap_info.max_bucket_scan_count = max_bucket_scan_count;
  voxelmap_info.voxel_resolution = resolution;
  check_error << cudaMallocAsync(&voxelmap_info_ptr, sizeof(VoxelMapInfo), stream);
  check_error << cudaMemcpyAsync(voxelmap_info_ptr, &voxelmap_info, sizeof(VoxelMapInfo), cudaMemcpyHostToDevice, stream);

  buckets = nullptr;

  num_points = nullptr;
  voxel_means = nullptr;
  voxel_covs = nullptr;
  voxel_intensities = nullptr;
}

GaussianVoxelMapGPU::~GaussianVoxelMapGPU() {
  check_error << cudaFreeAsync(voxelmap_info_ptr, 0);
  check_error << cudaFreeAsync(buckets, 0);
  check_error << cudaFreeAsync(num_points, 0);
  check_error << cudaFreeAsync(voxel_means, 0);
  check_error << cudaFreeAsync(voxel_covs, 0);
  check_error << cudaFreeAsync(voxel_intensities, 0);
}

void GaussianVoxelMapGPU::insert(const PointCloud& frame) {
  if (!frame.check_points_gpu() || !frame.check_covs_gpu()) {
    std::cerr << "error: GPU points/covs not allocated!!" << std::endl;
    abort();
  }

  create_bucket_table(stream, frame);
  check_error << cudaMallocAsync(&num_points, sizeof(int) * voxelmap_info.num_voxels, stream);
  check_error << cudaMallocAsync(&voxel_means, sizeof(Eigen::Vector3f) * voxelmap_info.num_voxels, stream);
  check_error << cudaMallocAsync(&voxel_covs, sizeof(Eigen::Matrix3f) * voxelmap_info.num_voxels, stream);
  check_error << cudaMallocAsync(&voxel_intensities, sizeof(float) * voxelmap_info.num_voxels, stream);
  check_error << cudaMemsetAsync(num_points, 0, sizeof(int) * voxelmap_info.num_voxels, stream);
  check_error << cudaMemsetAsync(voxel_means, 0, sizeof(Eigen::Vector3f) * voxelmap_info.num_voxels, stream);
  check_error << cudaMemsetAsync(voxel_covs, 0, sizeof(Eigen::Matrix3f) * voxelmap_info.num_voxels, stream);
  check_error << cudaMemsetAsync(voxel_intensities, 0, sizeof(float) * voxelmap_info.num_voxels, stream);

  uint64_t* voxel_means_fixed = nullptr;
  uint64_t* voxel_covs_fixed = nullptr;
  check_error << cudaMallocAsync(&voxel_means_fixed, sizeof(uint64_t) * 3 * voxelmap_info.num_voxels, stream);
  check_error << cudaMallocAsync(&voxel_covs_fixed, sizeof(uint64_t) * 9 * voxelmap_info.num_voxels, stream);
  check_error << cudaMemsetAsync(voxel_means_fixed, 0, sizeof(uint64_t) * 3 * voxelmap_info.num_voxels, stream);
  check_error << cudaMemsetAsync(voxel_covs_fixed, 0, sizeof(uint64_t) * 9 * voxelmap_info.num_voxels, stream);

  thrust::device_ptr<Eigen::Vector3f> points_ptr(frame.points_gpu);
  thrust::device_ptr<Eigen::Matrix3f> covs_ptr(frame.covs_gpu);
  if (frame.intensities_gpu) {
    thrust::device_ptr<float> ints_ptr(frame.intensities_gpu);
    thrust::for_each(
      thrust::cuda::par_nosync.on(stream),
      thrust::make_zip_iterator(thrust::make_tuple(points_ptr, covs_ptr, ints_ptr)),
      thrust::make_zip_iterator(thrust::make_tuple(points_ptr + frame.size(), covs_ptr + frame.size(), ints_ptr + frame.size())),
      accumulate_points_kernel(voxelmap_info_ptr, buckets, num_points, voxel_means_fixed, voxel_covs_fixed, voxel_intensities));
  } else {
    // If intensities are missing, fill voxel intensities with zeros while still accumulating geometry.
    auto zero_ints = thrust::make_constant_iterator<float>(0.0f);
    thrust::for_each(
      thrust::cuda::par_nosync.on(stream),
      thrust::make_zip_iterator(thrust::make_tuple(points_ptr, covs_ptr, zero_ints)),
      thrust::make_zip_iterator(thrust::make_tuple(points_ptr + frame.size(), covs_ptr + frame.size(), zero_ints + frame.size())),
      accumulate_points_kernel(voxelmap_info_ptr, buckets, num_points, voxel_means_fixed, voxel_covs_fixed, voxel_intensities));
  }

  thrust::for_each(
    thrust::cuda::par_nosync.on(stream),
    thrust::counting_iterator<int>(0),
    thrust::counting_iterator<int>(voxelmap_info.num_voxels),
    finalize_voxels_kernel(num_points, voxel_means_fixed, voxel_covs_fixed, voxel_means, voxel_covs, voxel_intensities));
  check_error << cudaFreeAsync(voxel_means_fixed, stream);
  check_error << cudaFreeAsync(voxel_covs_fixed, stream);
  cudaStreamSynchronize(stream);
}

void GaussianVoxelMapGPU::create_bucket_table(cudaStream_t stream, const PointCloud& frame) {
  if (frame.size() == 0) {
    voxelmap_info.num_voxels = 0;
    voxelmap_info.num_buckets = init_num_buckets;
    check_error << cudaMemcpyAsync(voxelmap_info_ptr, &voxelmap_info, sizeof(VoxelMapInfo), cudaMemcpyHostToDevice, stream);
    check_error << cudaFreeAsync(buckets, stream);
    check_error << cudaMallocAsync(&buckets, sizeof(VoxelBucket) * voxelmap_info.num_buckets, stream);
    std::vector<VoxelBucket> h_empty(voxelmap_info.num_buckets, {Eigen::Vector3i(0, 0, 0), -1});
    check_error << cudaMemcpyAsync(buckets, h_empty.data(), sizeof(VoxelBucket) * voxelmap_info.num_buckets, cudaMemcpyHostToDevice, stream);
    return;
  }

  // transform points(Vector3f) to voxel coords(Vector3i)
  Eigen::Vector3i* coords;
  check_error << cudaMallocAsync(&coords, sizeof(Eigen::Vector3i) * frame.size(), stream);
  thrust::transform(
    thrust::cuda::par_nosync.on(stream),
    thrust::device_ptr<Eigen::Vector3f>(frame.points_gpu),
    thrust::device_ptr<Eigen::Vector3f>(frame.points_gpu + frame.size()),
    coords,
    voxel_coord_kernel(voxelmap_info_ptr));

  // Sort coords deterministically
  thrust::sort(thrust::cuda::par_nosync.on(stream), coords, coords + frame.size(), Vector3iCoordLess());

  // Extract strictly unique coordinates
  auto unique_end = thrust::unique(thrust::cuda::par_nosync.on(stream), coords, coords + frame.size(), Vector3iCoordEqual());
  int num_voxels = thrust::distance(coords, unique_end);
  voxelmap_info.num_voxels = num_voxels;

  // Copy unique coords to host to build deterministic bucket table
  std::vector<Eigen::Vector3i> h_coords(num_voxels);
  check_error << cudaMemcpyAsync(h_coords.data(), coords, sizeof(Eigen::Vector3i) * num_voxels, cudaMemcpyDeviceToHost, stream);
  check_error << cudaStreamSynchronize(stream);
  check_error << cudaFreeAsync(coords, stream);

  int num_buckets = init_num_buckets;
  while (num_buckets < num_voxels * 2) {
    num_buckets *= 2;
  }

  std::vector<VoxelBucket> h_buckets;
  bool success = false;
  while (!success) {
    h_buckets.assign(num_buckets, {Eigen::Vector3i(0, 0, 0), -1});
    success = true;
    for (int i = 0; i < num_voxels; i++) {
      uint64_t hash = vector3i_hash(h_coords[i]);
      bool inserted = false;
      for (int step = 0; step < voxelmap_info.max_bucket_scan_count; step++) {
        uint64_t bucket_idx = (hash + step) % num_buckets;
        if (h_buckets[bucket_idx].second < 0) {
          h_buckets[bucket_idx] = {h_coords[i], i};
          inserted = true;
          break;
        }
      }
      if (!inserted) {
        num_buckets *= 2;
        success = false;
        break;
      }
    }
  }

  voxelmap_info.num_buckets = num_buckets;
  check_error << cudaFreeAsync(buckets, stream);
  check_error << cudaMallocAsync(&buckets, sizeof(VoxelBucket) * voxelmap_info.num_buckets, stream);
  check_error << cudaMemcpyAsync(buckets, h_buckets.data(), sizeof(VoxelBucket) * voxelmap_info.num_buckets, cudaMemcpyHostToDevice, stream);
  check_error << cudaMemcpyAsync(voxelmap_info_ptr, &voxelmap_info, sizeof(VoxelMapInfo), cudaMemcpyHostToDevice, stream);
}

void GaussianVoxelMapGPU::save_compact(const std::string& path) const {
  const Eigen::Vector3i INVALID_COORD = Eigen::Vector3i::Constant(std::numeric_limits<int>::max());

  std::vector<VoxelBucket> h_buckets(voxelmap_info.num_buckets);
  std::vector<Eigen::Vector3i> h_voxel_coords(voxelmap_info.num_voxels, INVALID_COORD);
  std::vector<int> h_num_points(voxelmap_info.num_voxels);
  std::vector<Eigen::Vector3f> h_voxel_means(voxelmap_info.num_voxels);
  std::vector<Eigen::Matrix3f> h_voxel_covs(voxelmap_info.num_voxels);
  std::vector<float> h_voxel_intensities(voxelmap_info.num_voxels);

  check_error << cudaMemcpyAsync(h_buckets.data(), buckets, sizeof(VoxelBucket) * voxelmap_info.num_buckets, cudaMemcpyDeviceToHost, 0);
  check_error << cudaMemcpyAsync(h_num_points.data(), num_points, sizeof(int) * voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, 0);
  check_error << cudaMemcpyAsync(h_voxel_means.data(), voxel_means, sizeof(Eigen::Vector3f) * voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, 0);
  check_error << cudaMemcpyAsync(h_voxel_covs.data(), voxel_covs, sizeof(Eigen::Matrix3f) * voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, 0);
  check_error << cudaMemcpyAsync(h_voxel_intensities.data(), voxel_intensities, sizeof(float) * voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, 0);

  std::vector<GaussianVoxelData> serial_voxels;
  serial_voxels.reserve(voxelmap_info.num_voxels);

  for (int i = 0; i < voxelmap_info.num_buckets; i++) {
    const auto& bucket = h_buckets[i];
    if (bucket.second < 0) {
      continue;
    }

    const auto& coord = bucket.first;
    const int voxel_index = bucket.second;

    if (voxel_index > voxelmap_info.num_voxels) {
      std::cerr << "error: voxel_index=" << voxel_index << " > num_voxels=" << voxelmap_info.num_voxels << std::endl;
      abort();
    }

    h_voxel_coords[voxel_index] = coord;
  }

  for (int i = 0; i < voxelmap_info.num_voxels; i++) {
    if (h_voxel_coords[i] == INVALID_COORD) {
      std::cerr << "error: h_voxel_coords[" << i << "] is not assigned!!" << std::endl;
      continue;
    }

    GaussianVoxel voxel;
    voxel.num_points = h_num_points[i];
    voxel.mean = h_voxel_means[i].homogeneous().cast<double>();
    voxel.cov.setZero();
    voxel.cov.topLeftCorner<3, 3>() = h_voxel_covs[i].cast<double>();
    voxel.intensity = h_voxel_intensities[i];
    serial_voxels.emplace_back(h_voxel_coords[i], voxel);
  }

  std::ofstream ofs(path);
  ofs << "compact " << 1 << std::endl;
  ofs << "resolution " << voxel_resolution() << std::endl;
  ofs << "lru_count " << 0 << std::endl;
  ofs << "lru_cycle " << 1 << std::endl;
  ofs << "lru_thresh " << 1 << std::endl;
  ofs << "voxel_bytes " << sizeof(GaussianVoxelData) << std::endl;
  ofs << "num_voxels " << serial_voxels.size() << std::endl;

  ofs.write(reinterpret_cast<const char*>(serial_voxels.data()), sizeof(GaussianVoxelData) * serial_voxels.size());
}

GaussianVoxelMapGPU::Ptr GaussianVoxelMapGPU::load(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "error: failed to open " << path << std::endl;
    return nullptr;
  }

  std::string token;
  bool compact;
  double resolution;
  int lru;
  int voxel_bytes;
  int num_voxels;

  ifs >> token >> compact;
  ifs >> token >> resolution;
  ifs >> token >> lru;
  ifs >> token >> lru;
  ifs >> token >> lru;
  ifs >> token >> voxel_bytes;
  ifs >> token >> num_voxels;
  std::getline(ifs, token);

  std::vector<GaussianVoxelData> flat_voxels(num_voxels);
  ifs.read(reinterpret_cast<char*>(flat_voxels.data()), sizeof(GaussianVoxelData) * num_voxels);

  std::vector<Eigen::Vector3i> h_coords(num_voxels);
  std::vector<int> h_num_points(num_voxels);
  std::vector<Eigen::Vector3f> h_voxel_means(num_voxels);
  std::vector<Eigen::Matrix3f> h_voxel_covs(num_voxels);
  std::vector<float> h_voxel_intensities(num_voxels);

  for (int i = 0; i < num_voxels; i++) {
    const auto& info_voxel = flat_voxels[i].uncompact();
    h_coords[i] = info_voxel->first.coord;
    h_num_points[i] = info_voxel->second.num_points;
    h_voxel_means[i] = info_voxel->second.mean.head<3>().cast<float>();
    h_voxel_covs[i] = info_voxel->second.cov.topLeftCorner<3, 3>().cast<float>();
    h_voxel_intensities[i] = info_voxel->second.intensity;
  }

  std::vector<VoxelBucket> h_buckets;
  const auto assign_buckets = [&](int num_buckets) {
    h_buckets.resize(num_buckets);
    for (int i = 0; i < num_buckets; i++) {
      h_buckets[i].first = Eigen::Vector3i(0, 0, 0);
      h_buckets[i].second = -1;
    }

    for (int i = 0; i < num_voxels; i++) {
      uint64_t hash = vector3i_hash(h_coords[i]);

      bool inserted = false;
      for (int j = 0; j < 10; j++) {
        uint64_t bucket_index = (hash + j) % num_buckets;
        auto& bucket = h_buckets[bucket_index];

        if (bucket.second < 0) {
          bucket.first = h_coords[i];
          bucket.second = i;
          inserted = true;
          break;
        }
      }

      if (!inserted) {
        return false;
      }
    }

    return true;
  };

  for (int num_buckets = 8192 * 4; num_buckets < (1 << 30); num_buckets *= 2) {
    if (assign_buckets(num_buckets)) {
      break;
    }
  }

  auto voxelmap = std::make_shared<GaussianVoxelMapGPU>(resolution, 8192, 10, 0.1);
  voxelmap->voxelmap_info.num_voxels = num_voxels;
  voxelmap->voxelmap_info.num_buckets = h_buckets.size();
  check_error << cudaMemcpyAsync(voxelmap->voxelmap_info_ptr, &voxelmap->voxelmap_info, sizeof(VoxelMapInfo), cudaMemcpyHostToDevice, 0);

  check_error << cudaMallocAsync(&voxelmap->buckets, sizeof(VoxelBucket) * h_buckets.size(), 0);
  check_error << cudaMallocAsync(&voxelmap->num_points, sizeof(int) * num_voxels, 0);
  check_error << cudaMallocAsync(&voxelmap->voxel_means, sizeof(Eigen::Vector3f) * num_voxels, 0);
  check_error << cudaMallocAsync(&voxelmap->voxel_covs, sizeof(Eigen::Matrix3f) * num_voxels, 0);
  check_error << cudaMallocAsync(&voxelmap->voxel_intensities, sizeof(float) * num_voxels, 0);
  check_error << cudaMemcpyAsync(voxelmap->buckets, h_buckets.data(), sizeof(VoxelBucket) * h_buckets.size(), cudaMemcpyHostToDevice, 0);
  check_error << cudaMemcpyAsync(voxelmap->num_points, h_num_points.data(), sizeof(int) * num_voxels, cudaMemcpyHostToDevice, 0);
  check_error << cudaMemcpyAsync(voxelmap->voxel_means, h_voxel_means.data(), sizeof(Eigen::Vector3f) * num_voxels, cudaMemcpyHostToDevice, 0);
  check_error << cudaMemcpyAsync(voxelmap->voxel_covs, h_voxel_covs.data(), sizeof(Eigen::Matrix3f) * num_voxels, cudaMemcpyHostToDevice, 0);
  check_error << cudaMemcpyAsync(voxelmap->voxel_intensities, h_voxel_intensities.data(), sizeof(float) * num_voxels, cudaMemcpyHostToDevice, 0);
  return voxelmap;
}

size_t GaussianVoxelMapGPU::memory_usage_gpu() const {
  return voxelmap_info.num_voxels * (sizeof(int) + sizeof(Eigen::Vector3f) + sizeof(Eigen::Matrix3f)) +
         voxelmap_info.num_buckets * sizeof(gtsam_points::VoxelBucket);
}

bool GaussianVoxelMapGPU::loaded_on_gpu() const {
  return buckets;
}

bool GaussianVoxelMapGPU::offload_gpu(CUstream_st* stream) {
  if (!buckets) {
    return false;
  }

  if (offloaded_buckets.empty()) {
    offloaded_buckets.resize(voxelmap_info.num_buckets);
    offloaded_num_points.resize(voxelmap_info.num_voxels);
    offloaded_voxel_means.resize(voxelmap_info.num_voxels);
    offloaded_voxel_covs.resize(voxelmap_info.num_voxels);

    check_error
      << cudaMemcpyAsync(offloaded_buckets.data(), buckets, sizeof(VoxelBucket) * voxelmap_info.num_buckets, cudaMemcpyDeviceToHost, stream);
    check_error << cudaMemcpyAsync(offloaded_num_points.data(), num_points, sizeof(int) * voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, stream);
    check_error << cudaMemcpyAsync(
      offloaded_voxel_means.data(),
      voxel_means,
      sizeof(Eigen::Vector3f) * voxelmap_info.num_voxels,
      cudaMemcpyDeviceToHost,
      stream);
    check_error
      << cudaMemcpyAsync(offloaded_voxel_covs.data(), voxel_covs, sizeof(Eigen::Matrix3f) * voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, stream);
  }

  check_error << cudaFreeAsync(buckets, stream);
  check_error << cudaFreeAsync(num_points, stream);
  check_error << cudaFreeAsync(voxel_means, stream);
  check_error << cudaFreeAsync(voxel_covs, stream);
  buckets = nullptr;
  num_points = nullptr;
  voxel_means = nullptr;
  voxel_covs = nullptr;

  return true;
}

bool GaussianVoxelMapGPU::reload_gpu(CUstream_st* stream) {
  if (buckets) {
    return false;
  }

  if (offloaded_buckets.empty()) {
    std::cerr << "error: offloaded buckets are empty!!" << std::endl;
    return false;
  }
  check_error << cudaMallocAsync(&buckets, sizeof(VoxelBucket) * voxelmap_info.num_buckets, stream);
  check_error << cudaMallocAsync(&num_points, sizeof(int) * voxelmap_info.num_voxels, stream);
  check_error << cudaMallocAsync(&voxel_means, sizeof(Eigen::Vector3f) * voxelmap_info.num_voxels, stream);
  check_error << cudaMallocAsync(&voxel_covs, sizeof(Eigen::Matrix3f) * voxelmap_info.num_voxels, stream);
  check_error << cudaMemcpyAsync(buckets, offloaded_buckets.data(), sizeof(VoxelBucket) * voxelmap_info.num_buckets, cudaMemcpyHostToDevice, stream);
  check_error << cudaMemcpyAsync(num_points, offloaded_num_points.data(), sizeof(int) * voxelmap_info.num_voxels, cudaMemcpyHostToDevice, stream);
  check_error
    << cudaMemcpyAsync(voxel_means, offloaded_voxel_means.data(), sizeof(Eigen::Vector3f) * voxelmap_info.num_voxels, cudaMemcpyHostToDevice, stream);
  check_error
    << cudaMemcpyAsync(voxel_covs, offloaded_voxel_covs.data(), sizeof(Eigen::Matrix3f) * voxelmap_info.num_voxels, cudaMemcpyHostToDevice, stream);

  return true;
}

std::vector<VoxelBucket> download_buckets(const GaussianVoxelMapGPU& voxelmap, CUstream_st* stream) {
  std::vector<VoxelBucket> buckets(voxelmap.voxelmap_info.num_buckets);
  check_error
    << cudaMemcpyAsync(buckets.data(), voxelmap.buckets, sizeof(VoxelBucket) * voxelmap.voxelmap_info.num_buckets, cudaMemcpyDeviceToHost, stream);
  return buckets;
}

std::vector<int> download_voxel_num_points(const GaussianVoxelMapGPU& voxelmap, CUstream_st* stream) {
  std::vector<int> num_points(voxelmap.voxelmap_info.num_voxels);
  check_error
    << cudaMemcpyAsync(num_points.data(), voxelmap.num_points, sizeof(int) * voxelmap.voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, stream);
  return num_points;
}

std::vector<Eigen::Vector3f> download_voxel_means(const GaussianVoxelMapGPU& voxelmap, CUstream_st* stream) {
  std::vector<Eigen::Vector3f> means(voxelmap.voxelmap_info.num_voxels);
  check_error << cudaMemcpyAsync(
    means.data(),
    voxelmap.voxel_means,
    sizeof(Eigen::Vector3f) * voxelmap.voxelmap_info.num_voxels,
    cudaMemcpyDeviceToHost,
    stream);
  return means;
}

std::vector<Eigen::Matrix3f> download_voxel_covs(const GaussianVoxelMapGPU& voxelmap, CUstream_st* stream) {
  std::vector<Eigen::Matrix3f> covs(voxelmap.voxelmap_info.num_voxels);
  check_error
    << cudaMemcpyAsync(covs.data(), voxelmap.voxel_covs, sizeof(Eigen::Matrix3f) * voxelmap.voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, stream);
  return covs;
}

std::vector<float> download_voxel_intensities(const GaussianVoxelMapGPU& vm, CUstream_st* stream) {
  std::vector<float> ints(vm.voxelmap_info.num_voxels);
  check_error << cudaMemcpyAsync(ints.data(), vm.voxel_intensities, sizeof(float) * vm.voxelmap_info.num_voxels, cudaMemcpyDeviceToHost, stream);
  return ints;
}

}  // namespace gtsam_points
