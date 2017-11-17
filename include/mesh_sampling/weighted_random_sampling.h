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
#include <memory>
#include <random>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class aiScene;
class aiMesh;
class aiFace;

namespace mesh_sampling
{

double triangle_area(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);
Eigen::Vector3f random_point_in_triangle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);


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
std::vector<SampleT> weighted_random_choice_indices(const std::vector<ProbaT>& probabilities, const size_t n)
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
std::vector<SampleT> weighted_random_choice(const std::vector<SampleT>& samples, const std::vector<ProbaT>& probabilities, const size_t n)
{
    const auto& indices = weighted_random_choice_indices(probabilities, n);
    std::vector<SampleT> vec(n);
    std::transform(indices.begin(), indices.end(), vec.begin(), [&samples](int index) { return samples[index]; });
    return vec;
}


template<typename T>
T randMToN(T M, T N)
{
    return M + (static_cast<T>(std::rand()) / ( RAND_MAX / (N-M) ) ) ;
}

/**
 * @brief Create pointcloud from MESH using weighted random sampling
 *
 * See the pyntcloud python library for more details:
 * https://github.com/daavoo/pyntcloud
 * https://github.com/daavoo/pyntcloud/blob/master/pyntcloud/samplers/s_mesh.py
 * https://medium.com/@daviddelaiglesiacastro/3f-point-cloud-generation-from-3f-triangular-mesh-bbb602ecf238
 */
  class WeightedRandomSampling
  {
    private:
      const aiScene *scene;

      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > v1_xyz,v2_xyz,v3_xyz;
      std::vector<double> areas;
      double total_area = 0;

      void process_triangle(const aiMesh* mesh, const aiFace& face);
      void process_mesh(const aiMesh* mesh);
      void process_scene(const aiScene *scene);
    public:
      WeightedRandomSampling(const aiScene* scene);
      /**
       * @brief Compute a weighted random sampling of the scene.
       *
       * @param N Number of samples
       * If N = 0, then the the number of triangles in the scene will be used to
       * generate samples.
       *
       * @return Sampled pointcloud from scene
       */
      std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> weighted_random_sampling(const size_t N = 0);
  };
} /* mesh_sampling */
