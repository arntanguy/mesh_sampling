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

#include "mesh_sampling/weighted_random_sampling.h"
#include <assimp/scene.h>  // Output data structure

namespace mesh_sampling
{
WeightedRandomSampling::WeightedRandomSampling(const aiScene* scene)
    : scene(scene)
{
  process_scene(scene);
}

std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
WeightedRandomSampling::weighted_random_sampling(const size_t N_samples)
{
  std::vector<double> probabilities;
  probabilities.reserve(areas.size());
  double total = 0;
  for (const auto& area : areas)
  {
    total += area / total_area;
    probabilities.push_back(area / total_area);
  }

  size_t N = N_samples;
  if(N_samples==0) N = areas.size();

  const auto random_idx =
      weighted_random_choice_indices<int, double>(probabilities, N);
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(
      new pcl::PointCloud<pcl::PointXYZ>(static_cast<uint32_t>(N),1));
  for (const auto idx : random_idx)
  {
    const Eigen::Vector3f point =
        random_point_in_triangle(v1_xyz[idx], v2_xyz[idx], v3_xyz[idx]);
    pcl::PointXYZ p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    cloud->push_back(p);
  }

  return cloud;
}

void WeightedRandomSampling::process_triangle(const aiMesh* mesh,
                                              const aiFace& face)
{
  // If face is a triangle
  if (face.mNumIndices == 3)
  {
    auto vertices = mesh->mVertices;
    auto fv1 = vertices[face.mIndices[0]];
    auto fv2 = vertices[face.mIndices[1]];
    auto fv3 = vertices[face.mIndices[2]];

    Eigen::Vector3f v1, v2, v3;
    v1 << fv1.x, fv1.y, fv1.z;
    v2 << fv2.x, fv2.y, fv2.z;
    v3 << fv3.x, fv3.y, fv3.z;
    v1_xyz.push_back(v1);
    v2_xyz.push_back(v2);
    v3_xyz.push_back(v3);
    const auto area = triangle_area(v1, v2, v3);
    areas.push_back(area);
    total_area += area;
  }
}

void WeightedRandomSampling::process_mesh(const aiMesh* mesh)
{
  if (!mesh->HasFaces()) return;

  // Count total number of triangles
  size_t total_triangles = 0;
  for (unsigned int i = 0; i < mesh->mNumFaces; ++i)
  {
    const auto face = mesh->mFaces[i];
    // If face is a triangle
    if (face.mNumIndices == 3)
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
  for (unsigned int i = 0; i < mesh->mNumFaces; ++i)
  {
    const auto face = mesh->mFaces[i];
    process_triangle(mesh, face);
  }
}

void WeightedRandomSampling::process_scene(const aiScene* scene)
{
  if (!scene->HasMeshes()) return;

  for (unsigned int i = 0; i < scene->mNumMeshes; ++i)
  {
    process_mesh(scene->mMeshes[i]);
  }
}

double triangle_area(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2,
                     const Eigen::Vector3f& v3)
{
  return 0.5 * (v2 - v1).cross(v3 - v1).norm();
}

/**
 * @brief Use barycentric coordinates to compute random point coordinates in a
 * triangle
 *
 * @param v1,v2,v3 : triangle vertices
 *
 * @return a random point in the triangle (v1,v2,v3)
 */
Eigen::Vector3f random_point_in_triangle(const Eigen::Vector3f& v1,
                                         const Eigen::Vector3f& v2,
                                         const Eigen::Vector3f& v3)
{
  float u = randMToN<float>(0, 1);
  float v = randMToN<float>(0, 1);
  if (u + v > 1)
  {
    u = 1 - u;
    v = 1 - v;
  }
  return (v1 * u) + (v2 * v) + ((1 - (u + v)) * v3);
}

} /* mesh_sampling */
