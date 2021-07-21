/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <boost/test/unit_test.hpp>

#include <iostream>
#include <memory>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/weighted_random_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#ifndef PROJECT_SOURCE_DIR
#  error "PROJECT_SOURCE_DIR must be defined to compile this file"
#endif

using namespace mesh_sampling;

// This is a very simple test that merely loads a sample CAD model,
// and samples N point from it.
// It only tests that the object can be loaded, and that the number of points
// after sampling is as expected
BOOST_AUTO_TEST_CASE(TestWeightedRandomSampling)
{
  const std::string model_path = PROJECT_SOURCE_DIR "/sample/suzanne.obj";
  size_t N = 1000;

  BOOST_REQUIRE_THROW(ASSIMPScene("wrong path.dae"), std::runtime_error);
  auto mesh = std::unique_ptr<ASSIMPScene>(new ASSIMPScene(model_path));

  std::cout << "Sampling " << N << " points from " << model_path << std::endl;
  WeightedRandomSampling<pcl::PointXYZ> sampling_xyz(mesh->scene());
  auto cloud_xyz = sampling_xyz.weighted_random_sampling(N);
  pcl::io::savePCDFileASCII("/tmp/example_xyz.pcd", *cloud_xyz);
  BOOST_REQUIRE(cloud_xyz->size() == N);

  WeightedRandomSampling<pcl::PointXYZRGB> sampling_rgb(mesh->scene());
  auto cloud_rgb = sampling_rgb.weighted_random_sampling(N);
  BOOST_REQUIRE(cloud_rgb->size() == N);

  WeightedRandomSampling<pcl::PointNormal> sampling_normal(mesh->scene());
  auto cloud_normal = sampling_normal.weighted_random_sampling(N);
  BOOST_REQUIRE(cloud_normal->size() == N);

  WeightedRandomSampling<pcl::PointXYZRGBNormal> sampling_rgb_normal(mesh->scene());
  auto cloud_rgb_normal = sampling_rgb_normal.weighted_random_sampling(N);
  BOOST_REQUIRE(cloud_rgb_normal->size() == N);
}
