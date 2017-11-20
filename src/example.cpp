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

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/weighted_random_sampling.h>

using namespace mesh_sampling;

void help()
{
  std::cout << "Usage: ./example path_to_model" << std::endl;
  exit(1);
}

int main(int argc, char** argv)
{
  std::string model_path = "";
  if (argc > 1)
  {
    if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")
    {
      help();
    }
    else
    {
      model_path = argv[1];
    }
  }
  else
  {
    help();
  }

  // ASSIMPScene loader should be used and kept in scope for as long as the mesh
  // is needed.
  // The default constructor loads the mesh with all the required
  // post-processing according to point type (normal computation...)
  std::unique_ptr<ASSIMPScene> mesh = nullptr;
  try
  {
    mesh = std::unique_ptr<ASSIMPScene>(new ASSIMPScene(model_path));
  }
  catch (std::runtime_error& e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  WeightedRandomSampling<pcl::PointXYZ> sampling_xyz(mesh->scene());
  auto cloud_xyz = sampling_xyz.weighted_random_sampling();
  pcl::io::savePCDFileASCII("/tmp/example_xyz.pcd", *cloud_xyz);

  WeightedRandomSampling<pcl::PointXYZRGB> sampling_rgb(mesh->scene());
  auto cloud_rgb = sampling_rgb.weighted_random_sampling();
  pcl::io::savePCDFileASCII("/tmp/example_rgb.pcd", *cloud_rgb);

  WeightedRandomSampling<pcl::PointNormal> sampling_normal(mesh->scene());
  auto cloud_normal = sampling_normal.weighted_random_sampling();
  pcl::io::savePCDFileASCII("/tmp/example_normal.pcd", *cloud_normal);

  WeightedRandomSampling<pcl::PointXYZRGBNormal> sampling_rgb_normal(
      mesh->scene());
  auto cloud_rgb_normal = sampling_rgb_normal.weighted_random_sampling();
  pcl::io::savePCDFileASCII("/tmp/example_rgb_normal.pcd", *cloud_rgb_normal);
}
