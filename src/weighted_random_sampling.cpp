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

namespace mesh_sampling
{

// Explicit instantiation for common types
template class WeightedRandomSampling<pcl::PointXYZ>;
template class WeightedRandomSampling<pcl::PointXYZRGB>;
template class WeightedRandomSampling<pcl::PointNormal>;
template class WeightedRandomSampling<pcl::PointXYZRGBNormal>;


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
