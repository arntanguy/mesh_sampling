// Copyright 2017 CNRS-UM LIRMM
// Copyright 2017 Arnaud TANGUY <arnaud.tanguy@lirmm.fr>
//
// This file is part of mesh_sampling.
//
// mesh_sampling is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// mesh_sampling is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with mesh_sampling.  If not, see <http://www.gnu.org/licenses/>.

#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <assimp/scene.h> // Output data structure
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>
#include <vector>

namespace mesh_sampling
{
double triangle_area(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2, const Eigen::Vector3f & v3);
Eigen::Vector3f random_point_in_triangle(const Eigen::Vector3f & v1,
                                         const Eigen::Vector3f & v2,
                                         const Eigen::Vector3f & v3);

/**
 * @brief Generates indices based on probabilities
 *
 * @param probabilities The probabilities associated with each entry in samples.
 * @param n number of generated samples
 *
 * @return return n sampled indices from the probability distribution. Each
 * index will be between [0, probabilities.size()[
 */
template<typename SampleT, typename ProbaT>
std::vector<SampleT> weighted_random_choice_indices(const std::vector<ProbaT> & probabilities, const size_t n)
{
  std::default_random_engine generator;
  std::discrete_distribution<int> distribution(probabilities.begin(), probabilities.end());

  std::vector<int> indices(n);
  std::generate(indices.begin(), indices.end(), [&generator, &distribution]() { return distribution(generator); });
  return indices;
}

/**
 * @brief Implementation of python's numpy.random.choice
 * Generates a random sample from a given 1-D array
 * Since range lazy iterators are not available in C++11, using this function
 * might necessitate generating a large samples array, whereby python would be
 * able to take advantage of lazy generation.
 *
 * @param samples a random sample is generated from its elements
 * @param probabilities The probabilities associated with each entry in samples.
 * @param n number of samples
 *
 * @return The generated random samples
 */
template<typename SampleT, typename ProbaT>
std::vector<SampleT> weighted_random_choice(const std::vector<SampleT> & samples,
                                            const std::vector<ProbaT> & probabilities,
                                            const size_t n)
{
  const auto & indices = weighted_random_choice_indices(probabilities, n);
  std::vector<SampleT> vec(n);
  std::transform(indices.begin(), indices.end(), vec.begin(), [&samples](int index) { return samples[index]; });
  return vec;
}

template<typename T>
T randMToN(T M, T N)
{
  return M + (static_cast<T>(std::rand()) / (RAND_MAX / (N - M)));
}

/**
 * @brief Create pointcloud from MESH using weighted random sampling
 *
 * See the pyntcloud python library for more details:
 * https://github.com/daavoo/pyntcloud
 * https://github.com/daavoo/pyntcloud/blob/master/pyntcloud/samplers/s_mesh.py
 * https://medium.com/@daviddelaiglesiacastro/3f-point-cloud-generation-from-3f-triangular-mesh-bbb602ecf238
 */
template<typename PointT>
class WeightedRandomSampling
{
  using CloudT = pcl::PointCloud<PointT>;

private:
  const aiScene * scene;

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> v1_xyz, v2_xyz, v3_xyz;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> v1_rgb, v2_rgb, v3_rgb;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> v1_normal, v2_normal, v3_normal;
  std::vector<double> areas;
  double total_area = 0;

  void process_triangle(const aiMesh * mesh, const aiFace & face)
  {
    // If face is a triangle
    if(face.mNumIndices == 3)
    {
      auto vertices = mesh->mVertices;
      const auto & fv1 = vertices[face.mIndices[0]];
      const auto & fv2 = vertices[face.mIndices[1]];
      const auto & fv3 = vertices[face.mIndices[2]];

      Eigen::Vector3f v1, v2, v3;
      v1 << fv1.x, fv1.y, fv1.z;
      v2 << fv2.x, fv2.y, fv2.z;
      v3 << fv3.x, fv3.y, fv3.z;
      v1_xyz.push_back(v1);
      v2_xyz.push_back(v2);
      v3_xyz.push_back(v3);

      if(mesh->HasVertexColors(0))
      {
        auto color = mesh->mColors[0];
        const auto & cv1 = color[face.mIndices[0]];
        const auto & cv2 = color[face.mIndices[1]];
        const auto & cv3 = color[face.mIndices[2]];
        v1_rgb.emplace_back(Eigen::Vector3f{cv1.r, cv1.g, cv1.b});
        v2_rgb.emplace_back(Eigen::Vector3f{cv2.r, cv2.g, cv2.b});
        v3_rgb.emplace_back(Eigen::Vector3f{cv3.r, cv3.g, cv3.b});
      }
      else
      {
        v1_rgb.emplace_back(255 * Eigen::Vector3f::Ones());
        v2_rgb.emplace_back(255 * Eigen::Vector3f::Ones());
        v3_rgb.emplace_back(255 * Eigen::Vector3f::Ones());
      }

      if(mesh->HasNormals())
      {
        auto normals = mesh->mNormals;
        const auto & n1 = normals[face.mIndices[0]];
        const auto & n2 = normals[face.mIndices[1]];
        const auto & n3 = normals[face.mIndices[2]];
        v1_normal.emplace_back(Eigen::Vector3f{n1.x, n1.y, n1.z});
        v2_normal.emplace_back(Eigen::Vector3f{n2.x, n2.y, n2.z});
        v3_normal.emplace_back(Eigen::Vector3f{n3.x, n3.y, n3.z});
      }
      else
      {
        // Should always have normals if loaded with ASSIMPScene loader
        v1_normal.emplace_back(Eigen::Vector3f{0., 0., 0.});
        v2_normal.emplace_back(Eigen::Vector3f{0., 0., 0.});
        v3_normal.emplace_back(Eigen::Vector3f{0., 0., 0.});
      }

      const auto area = triangle_area(v1, v2, v3);
      areas.push_back(area);
      total_area += area;
    }
  }

  void process_mesh(const aiMesh * mesh)
  {
    if(!mesh->HasFaces()) return;

    // Count total number of triangles
    size_t total_triangles = 0;
    for(unsigned int i = 0; i < mesh->mNumFaces; ++i)
    {
      const auto face = mesh->mFaces[i];
      // If face is a triangle
      if(face.mNumIndices == 3)
      {
        ++total_triangles;
      }
    }

    // Reserve memory for all triangles
    v1_xyz.reserve(total_triangles);
    v2_xyz.reserve(total_triangles);
    v3_xyz.reserve(total_triangles);
    areas.reserve(total_triangles);

    // Process each triangle
    for(unsigned int i = 0; i < mesh->mNumFaces; ++i)
    {
      const auto face = mesh->mFaces[i];
      process_triangle(mesh, face);
    }
  }

  void process_scene(const aiScene * scene)
  {
    if(!scene->HasMeshes()) return;

    for(unsigned int i = 0; i < scene->mNumMeshes; ++i)
    {
      process_mesh(scene->mMeshes[i]);
    }
  }

public:
  WeightedRandomSampling(const aiScene * scene) : scene(scene)
  {
    process_scene(scene);
  }

  /**
   * @brief This function computes and fill the point color only if the PointT
   * supports it (PointXYZRGB...)
   *
   * @param[out] p point to fill
   * @param[in] idx triangle id
   *
   * @return
   */
  template<class Q = PointT>
  typename std::enable_if<pcl::traits::has_color<Q>::value, void>::type fill_color(PointT & p, const size_t idx)
  {
    // HAS COLOR
    const Eigen::Vector3f color = random_point_in_triangle(v1_rgb[idx], v2_rgb[idx], v3_rgb[idx]);
    p.r = static_cast<uint8_t>(color.x());
    p.g = static_cast<uint8_t>(color.y());
    p.b = static_cast<uint8_t>(color.z());
  }

  // Does nothing if the PointT does not support colors
  template<class Q = PointT>
  typename std::enable_if<!pcl::traits::has_color<Q>::value, void>::type fill_color(PointT & /*p*/,
                                                                                    const size_t /*idx*/)
  {
  }

  /**
   * @brief This function computes and fill the point normals only if the PointT
   * supports it (PointNormal, PointXYZRGBNormal...)
   *
   * @param[out] p point to fill
   * @param[in] idx triangle id
   *
   * @return void
   */
  template<class Q = PointT>
  typename std::enable_if<pcl::traits::has_normal<Q>::value, void>::type fill_normal(PointT & p, const size_t idx)
  {
    const auto & n1 = v1_normal[idx];
    const auto & n2 = v2_normal[idx];
    const auto & n3 = v3_normal[idx];
    Eigen::Vector3f n = (n1 + n2 + n3).normalized();

    p.normal_x = n.x();
    p.normal_y = n.y();
    p.normal_z = n.z();
  }

  // Does nothing if the PointT does not support colors
  template<class Q = PointT>
  typename std::enable_if<!pcl::traits::has_normal<Q>::value, void>::type fill_normal(PointT & /*p*/,
                                                                                      const size_t /*idx*/)
  {
  }

  /**
   * @brief Compute a weighted random sampling of the scene.
   *
   * @param N Number of samples
   * If N = 0, then the the number of triangles in the scene will be used to
   * generate samples.
   *
   * @return Sampled pointcloud from scene
   */
  std::unique_ptr<CloudT> weighted_random_sampling(const size_t N_samples = 0)
  {
    std::vector<double> probabilities;
    probabilities.reserve(areas.size());
    double total = 0;
    for(const auto & area : areas)
    {
      total += area / total_area;
      probabilities.push_back(area / total_area);
    }

    size_t N = N_samples;
    if(N_samples == 0) N = areas.size();

    const auto random_idx = weighted_random_choice_indices<int, double>(probabilities, N);
    std::unique_ptr<CloudT> cloud(new CloudT());
    cloud->reserve(static_cast<uint32_t>(N));
    for(const auto idx : random_idx)
    {
      const Eigen::Vector3f point = random_point_in_triangle(v1_xyz[idx], v2_xyz[idx], v3_xyz[idx]);
      PointT p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      fill_color(p, idx);
      fill_normal(p, idx);
      cloud->push_back(p);
    }

    return cloud;
  }
};
} // namespace mesh_sampling
