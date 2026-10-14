#include <iostream>
#include <vector>
#include <cmath>
#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/HessianFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/cuda_stream.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

namespace gtsam_points {

static std::shared_ptr<PointCloudCPU> create_synthetic_cloud(int num_points = 10000) {
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Matrix3f> covs;
  std::vector<Eigen::Vector3f> normals;
  std::vector<float> intensities;
  points.reserve(num_points);
  covs.reserve(num_points);
  normals.reserve(num_points);
  intensities.reserve(num_points);

  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) * 0.01;
    float x = static_cast<float>(std::sin(t) * (10.0 + (i % 20) * 0.5));
    float y = static_cast<float>(std::cos(t) * (10.0 + (i % 20) * 0.5));
    float z = static_cast<float>((i % 50) * 0.1 - 2.5);

    points.emplace_back(x, y, z);

    Eigen::Matrix3f cov = Eigen::Matrix3f::Identity() * 0.01f;
    cov(0, 0) = 0.05f + (i % 7) * 0.005f;
    cov(1, 1) = 0.03f + (i % 11) * 0.003f;
    cov(2, 2) = 0.01f;
    covs.emplace_back(cov);

    Eigen::Vector3f n(0.0f, 0.0f, 1.0f);
    normals.emplace_back(n);

    intensities.emplace_back(static_cast<float>(i % 256));
  }

  auto cloud = std::make_shared<PointCloudCPU>(points);
  cloud->add_covs(covs);
  cloud->add_normals(normals);
  cloud->add_intensities(intensities);
  return cloud;
}

TEST(DeterminismTest, VoxelMapDeterminism) {
  auto cpu_cloud = create_synthetic_cloud(15000);
  CUDAStream stream1;
  CUDAStream stream2;

  auto gpu_cloud1 = PointCloudGPU::clone(*cpu_cloud, stream1);
  auto gpu_cloud2 = PointCloudGPU::clone(*cpu_cloud, stream2);

  double resolution = 0.5;
  auto voxelmap1 = std::make_shared<GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, stream1);
  auto voxelmap2 = std::make_shared<GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, stream2);

  voxelmap1->insert(*gpu_cloud1);
  voxelmap2->insert(*gpu_cloud2);

  int num_voxels1 = voxelmap1->voxelmap_info.num_voxels;
  int num_voxels2 = voxelmap2->voxelmap_info.num_voxels;
  int num_buckets1 = voxelmap1->voxelmap_info.num_buckets;
  int num_buckets2 = voxelmap2->voxelmap_info.num_buckets;

  std::cout << "Run 1: buckets=" << num_buckets1 << ", voxels=" << num_voxels1 << std::endl;
  std::cout << "Run 2: buckets=" << num_buckets2 << ", voxels=" << num_voxels2 << std::endl;

  EXPECT_EQ(num_buckets1, num_buckets2);
  EXPECT_EQ(num_voxels1, num_voxels2);

  std::vector<VoxelBucket> h_buckets1(num_buckets1);
  std::vector<VoxelBucket> h_buckets2(num_buckets2);
  cudaMemcpy(h_buckets1.data(), voxelmap1->buckets, sizeof(VoxelBucket) * num_buckets1, cudaMemcpyDeviceToHost);
  cudaMemcpy(h_buckets2.data(), voxelmap2->buckets, sizeof(VoxelBucket) * num_buckets2, cudaMemcpyDeviceToHost);

  int bucket_mismatches = 0;
  for (int i = 0; i < num_buckets1; ++i) {
    if (h_buckets1[i].second != h_buckets2[i].second ||
        h_buckets1[i].first != h_buckets2[i].first) {
      if (bucket_mismatches < 5) {
        std::cout << "Mismatch at bucket " << i << ": Run1=("
                  << h_buckets1[i].first.transpose() << ", " << h_buckets1[i].second << ") Run2=("
                  << h_buckets2[i].first.transpose() << ", " << h_buckets2[i].second << ")" << std::endl;
      }
      bucket_mismatches++;
    }
  }
  std::cout << "Bucket mismatches: " << bucket_mismatches << " / " << num_buckets1 << std::endl;

  std::vector<Eigen::Vector3f> means1(num_voxels1), means2(num_voxels2);
  std::vector<Eigen::Matrix3f> covs1(num_voxels1), covs2(num_voxels2);
  cudaMemcpy(means1.data(), voxelmap1->voxel_means, sizeof(Eigen::Vector3f) * num_voxels1, cudaMemcpyDeviceToHost);
  cudaMemcpy(means2.data(), voxelmap2->voxel_means, sizeof(Eigen::Vector3f) * num_voxels2, cudaMemcpyDeviceToHost);
  cudaMemcpy(covs1.data(), voxelmap1->voxel_covs, sizeof(Eigen::Matrix3f) * num_voxels1, cudaMemcpyDeviceToHost);
  cudaMemcpy(covs2.data(), voxelmap2->voxel_covs, sizeof(Eigen::Matrix3f) * num_voxels2, cudaMemcpyDeviceToHost);

  int mean_mismatches = 0;
  double max_mean_diff = 0.0;
  for (int i = 0; i < std::min(num_voxels1, num_voxels2); ++i) {
    double diff = (means1[i] - means2[i]).norm();
    if (diff > 0.0) {
      mean_mismatches++;
      max_mean_diff = std::max(max_mean_diff, diff);
    }
  }
  std::cout << "Mean mismatches: " << mean_mismatches << " / " << std::min(num_voxels1, num_voxels2)
            << " (max diff=" << max_mean_diff << ")" << std::endl;

  EXPECT_EQ(bucket_mismatches, 0);
  EXPECT_EQ(mean_mismatches, 0);
}

TEST(DeterminismTest, FactorLinearizationDeterminism) {
  auto target_cpu = create_synthetic_cloud(12000);
  auto source_cpu = create_synthetic_cloud(10000);

  CUDAStream stream;
  auto target_gpu = PointCloudGPU::clone(*target_cpu, stream);
  auto source_gpu = PointCloudGPU::clone(*source_cpu, stream);

  auto voxelmap = std::make_shared<GaussianVoxelMapGPU>(0.5f, 8192 * 2, 10, 1e-3, stream);
  voxelmap->insert(*target_gpu);

  auto temp_buffer1 = std::make_shared<TempBufferManager>();
  auto temp_buffer2 = std::make_shared<TempBufferManager>();

  gtsam::Pose3 fixed_target_pose;
  gtsam::Key source_key = 1;

  IntegratedVGICPFactorGPU factor1(fixed_target_pose, source_key, voxelmap, source_gpu, stream, temp_buffer1);
  IntegratedVGICPFactorGPU factor2(fixed_target_pose, source_key, voxelmap, source_gpu, stream, temp_buffer2);

  factor1.set_enable_surface_validation(true);
  factor2.set_enable_surface_validation(true);

  gtsam::Values values;
  values.insert(source_key, gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.015), gtsam::Point3(0.05, -0.03, 0.02)));

  auto g_factor1 = factor1.linearize(values);
  auto g_factor2 = factor2.linearize(values);

  auto h_factor1 = std::dynamic_pointer_cast<gtsam::HessianFactor>(g_factor1);
  auto h_factor2 = std::dynamic_pointer_cast<gtsam::HessianFactor>(g_factor2);

  ASSERT_TRUE(h_factor1 != nullptr);
  ASSERT_TRUE(h_factor2 != nullptr);

  auto H1 = h_factor1->information();
  auto H2 = h_factor2->information();
  auto g1 = h_factor1->linearTerm();
  auto g2 = h_factor2->linearTerm();
  auto err1 = h_factor1->constantTerm();
  auto err2 = h_factor2->constantTerm();

  double H_diff = (H1 - H2).norm();
  double g_diff = (g1 - g2).norm();
  double err_diff = std::abs(err1 - err2);

  std::cout << "H_diff: " << H_diff << std::endl;
  std::cout << "g_diff: " << g_diff << std::endl;
  std::cout << "err_diff: " << err_diff << std::endl;

  EXPECT_EQ(H_diff, 0.0);
  EXPECT_EQ(g_diff, 0.0);
  EXPECT_EQ(err_diff, 0.0);
}

TEST(DeterminismTest, CrossVoxelMapFactorLinearization) {
  auto target_cpu = create_synthetic_cloud(12000);
  auto source_cpu = create_synthetic_cloud(10000);

  CUDAStream stream1;
  CUDAStream stream2;
  auto target_gpu1 = PointCloudGPU::clone(*target_cpu, stream1);
  auto target_gpu2 = PointCloudGPU::clone(*target_cpu, stream2);
  auto source_gpu = PointCloudGPU::clone(*source_cpu, stream1);

  // Two independent voxelmaps created from the same target cloud
  auto voxelmap1 = std::make_shared<GaussianVoxelMapGPU>(0.5f, 8192 * 2, 10, 1e-3, stream1);
  auto voxelmap2 = std::make_shared<GaussianVoxelMapGPU>(0.5f, 8192 * 2, 10, 1e-3, stream2);
  voxelmap1->insert(*target_gpu1);
  voxelmap2->insert(*target_gpu2);

  auto temp_buffer1 = std::make_shared<TempBufferManager>();
  auto temp_buffer2 = std::make_shared<TempBufferManager>();

  gtsam::Pose3 fixed_target_pose;
  gtsam::Key source_key = 1;

  IntegratedVGICPFactorGPU factor1(fixed_target_pose, source_key, voxelmap1, source_gpu, stream1, temp_buffer1);
  IntegratedVGICPFactorGPU factor2(fixed_target_pose, source_key, voxelmap2, source_gpu, stream2, temp_buffer2);

  factor1.set_enable_surface_validation(true);
  factor2.set_enable_surface_validation(true);

  gtsam::Values values;
  values.insert(source_key, gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.015), gtsam::Point3(0.05, -0.03, 0.02)));

  auto g_factor1 = factor1.linearize(values);
  auto g_factor2 = factor2.linearize(values);

  auto h_factor1 = std::dynamic_pointer_cast<gtsam::HessianFactor>(g_factor1);
  auto h_factor2 = std::dynamic_pointer_cast<gtsam::HessianFactor>(g_factor2);

  ASSERT_TRUE(h_factor1 != nullptr);
  ASSERT_TRUE(h_factor2 != nullptr);

  auto H1 = h_factor1->information();
  auto H2 = h_factor2->information();
  auto g1 = h_factor1->linearTerm();
  auto g2 = h_factor2->linearTerm();
  auto err1 = h_factor1->constantTerm();
  auto err2 = h_factor2->constantTerm();

  double H_diff = (H1 - H2).norm();
  double g_diff = (g1 - g2).norm();
  double err_diff = std::abs(err1 - err2);

  std::cout << "Cross-VoxelMap H_diff: " << H_diff << std::endl;
  std::cout << "Cross-VoxelMap g_diff: " << g_diff << std::endl;
  std::cout << "Cross-VoxelMap err_diff: " << err_diff << std::endl;

  EXPECT_EQ(H_diff, 0.0);
  EXPECT_EQ(g_diff, 0.0);
  EXPECT_EQ(err_diff, 0.0);
}

TEST(DeterminismTest, NonlinearFactorSetGPULinearization) {
  LinearizationHook::register_hook([] { return create_nonlinear_factor_set_gpu(); });

  auto target_cpu = create_synthetic_cloud(12000);
  auto source_cpu = create_synthetic_cloud(10000);

  CUDAStream stream;
  auto target_gpu = PointCloudGPU::clone(*target_cpu, stream);
  auto source_gpu = PointCloudGPU::clone(*source_cpu, stream);

  auto voxelmap1 = std::make_shared<GaussianVoxelMapGPU>(0.5f, 8192 * 2, 10, 1e-3, stream);
  auto voxelmap2 = std::make_shared<GaussianVoxelMapGPU>(0.5f, 8192 * 2, 10, 1e-3, stream);
  voxelmap1->insert(*target_gpu);
  voxelmap2->insert(*target_gpu);

  auto temp_buffer1 = std::make_shared<TempBufferManager>();
  auto temp_buffer2 = std::make_shared<TempBufferManager>();

  gtsam::Pose3 fixed_target_pose;
  gtsam::Key source_key = 1;

  auto factor1 = gtsam::make_shared<IntegratedVGICPFactorGPU>(fixed_target_pose, source_key, voxelmap1, source_gpu, stream, temp_buffer1);
  auto factor2 = gtsam::make_shared<IntegratedVGICPFactorGPU>(fixed_target_pose, source_key, voxelmap2, source_gpu, stream, temp_buffer2);

  factor1->set_enable_surface_validation(true);
  factor2->set_enable_surface_validation(true);

  gtsam::Values values;
  values.insert(source_key, gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.015), gtsam::Point3(0.05, -0.03, 0.02)));

  // Linearize through NonlinearFactorSetGPU
  auto factor_set1 = create_nonlinear_factor_set_gpu();
  factor_set1->add(factor1);
  factor_set1->linearize(values);
  auto linear1 = factor_set1->calc_linear_factors(values);

  auto factor_set2 = create_nonlinear_factor_set_gpu();
  factor_set2->add(factor2);
  factor_set2->linearize(values);
  auto linear2 = factor_set2->calc_linear_factors(values);

  ASSERT_EQ(linear1.size(), 1);
  ASSERT_EQ(linear2.size(), 1);

  auto h_factor1 = std::dynamic_pointer_cast<gtsam::HessianFactor>(linear1[0]);
  auto h_factor2 = std::dynamic_pointer_cast<gtsam::HessianFactor>(linear2[0]);
  ASSERT_TRUE(h_factor1 != nullptr);
  ASSERT_TRUE(h_factor2 != nullptr);

  auto H1 = h_factor1->information();
  auto H2 = h_factor2->information();
  auto g1 = h_factor1->linearTerm();
  auto g2 = h_factor2->linearTerm();
  auto err1 = h_factor1->constantTerm();
  auto err2 = h_factor2->constantTerm();

  double H_diff = (H1 - H2).norm();
  double g_diff = (g1 - g2).norm();
  double err_diff = std::abs(err1 - err2);

  std::cout << "NonlinearFactorSetGPU H_diff: " << H_diff << std::endl;
  std::cout << "NonlinearFactorSetGPU g_diff: " << g_diff << std::endl;
  std::cout << "NonlinearFactorSetGPU err_diff: " << err_diff << std::endl;

  EXPECT_EQ(H_diff, 0.0);
  EXPECT_EQ(g_diff, 0.0);
  EXPECT_EQ(err_diff, 0.0);
}

TEST(DeterminismTest, RealDataVoxelMapDeterminism) {
  std::string path = "/data/glim_results/det_short_patched_1/000000/points_compact.bin";
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    std::cout << "Could not open " << path << ", skipping real data test" << std::endl;
    return;
  }

  ifs.seekg(0, std::ios::end);
  size_t bytes = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  size_t num_points = bytes / sizeof(Eigen::Vector3f);
  std::vector<Eigen::Vector3f> points(num_points);
  ifs.read(reinterpret_cast<char*>(points.data()), bytes);
  std::cout << "Loaded " << num_points << " points from " << path << std::endl;

  std::vector<Eigen::Matrix3f> covs(num_points, Eigen::Matrix3f::Identity() * 0.01f);
  std::vector<Eigen::Vector3f> normals(num_points, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

  auto cpu_cloud = std::make_shared<PointCloudCPU>(points);
  cpu_cloud->add_covs(covs);
  cpu_cloud->add_normals(normals);

  CUDAStream stream1;
  CUDAStream stream2;
  auto gpu_cloud1 = PointCloudGPU::clone(*cpu_cloud, stream1);
  auto gpu_cloud2 = PointCloudGPU::clone(*cpu_cloud, stream2);

  double resolution = 0.5;
  auto voxelmap1 = std::make_shared<GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, stream1);
  auto voxelmap2 = std::make_shared<GaussianVoxelMapGPU>(resolution, 8192 * 2, 10, 1e-3, stream2);

  voxelmap1->insert(*gpu_cloud1);
  voxelmap2->insert(*gpu_cloud2);

  std::cout << "Real data Run 1: buckets=" << voxelmap1->voxelmap_info.num_buckets << ", voxels=" << voxelmap1->voxelmap_info.num_voxels << std::endl;
  std::cout << "Real data Run 2: buckets=" << voxelmap2->voxelmap_info.num_buckets << ", voxels=" << voxelmap2->voxelmap_info.num_voxels << std::endl;

  EXPECT_EQ(voxelmap1->voxelmap_info.num_voxels, voxelmap2->voxelmap_info.num_voxels);

  // Now linearize a binary factor with these two voxelmaps
  auto temp_buffer1 = std::make_shared<TempBufferManager>();
  auto temp_buffer2 = std::make_shared<TempBufferManager>();

  gtsam::Key target_key = 0;
  gtsam::Key source_key = 1;

  auto factor1 = gtsam::make_shared<IntegratedVGICPFactorGPU>(target_key, source_key, voxelmap1, gpu_cloud1, stream1, temp_buffer1);
  auto factor2 = gtsam::make_shared<IntegratedVGICPFactorGPU>(target_key, source_key, voxelmap2, gpu_cloud2, stream2, temp_buffer2);

  factor1->set_enable_surface_validation(true);
  factor2->set_enable_surface_validation(true);

  gtsam::Values values;
  values.insert(target_key, gtsam::Pose3());
  values.insert(source_key, gtsam::Pose3(gtsam::Rot3::RzRyRx(0.005, -0.008, 0.004), gtsam::Point3(0.02, -0.01, 0.005)));

  auto factor_set1 = create_nonlinear_factor_set_gpu();
  factor_set1->add(factor1);
  factor_set1->linearize(values);
  auto linear1 = factor_set1->calc_linear_factors(values);

  auto factor_set2 = create_nonlinear_factor_set_gpu();
  factor_set2->add(factor2);
  factor_set2->linearize(values);
  auto linear2 = factor_set2->calc_linear_factors(values);

  auto h_factor1 = std::dynamic_pointer_cast<gtsam::HessianFactor>(linear1[0]);
  auto h_factor2 = std::dynamic_pointer_cast<gtsam::HessianFactor>(linear2[0]);

  auto H1 = h_factor1->information();
  auto H2 = h_factor2->information();
  auto g1 = h_factor1->linearTerm();
  auto g2 = h_factor2->linearTerm();
  auto err1 = h_factor1->constantTerm();
  auto err2 = h_factor2->constantTerm();

  double H_diff = (H1 - H2).norm();
  double g_diff = (g1 - g2).norm();
  double err_diff = std::abs(err1 - err2);

  std::cout << "Real Data Cross-VoxelMap H_diff: " << H_diff << std::endl;
  std::cout << "Real Data Cross-VoxelMap g_diff: " << g_diff << std::endl;
  std::cout << "Real Data Cross-VoxelMap err_diff: " << err_diff << std::endl;

  EXPECT_EQ(H_diff, 0.0);
  EXPECT_EQ(g_diff, 0.0);
  EXPECT_EQ(err_diff, 0.0);
}

}  // namespace gtsam_points
