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

int main(int /* argc */, char** /* argv */)
{
  std::string model_path = "../../getafe.stl";

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

  WeightedRandomSampling sampling(mesh->scene());
  auto cloud = sampling.weighted_random_sampling();

  pcl::io::savePCDFileASCII("/tmp/test_pcd.pcd", *cloud);
  pcl::io::savePLYFileASCII("/tmp/test.ply", *cloud);
}
