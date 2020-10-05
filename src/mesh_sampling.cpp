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
#include <algorithm>
#include <iterator>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/weighted_random_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace mesh_sampling;

template<typename PointT>
void create_cloud(const aiScene * scene, unsigned N, const bfs::path & out_path, bool binary_mode = false)
{
  WeightedRandomSampling<PointT> sampler(scene);
  auto cloud = sampler.weighted_random_sampling(N);

  auto extension = out_path.extension();
  if(extension == ".pcd")
  {
    pcl::io::savePCDFile(out_path.string(), *cloud, binary_mode);
  }
  else if(extension == ".ply")
  {
    pcl::io::savePLYFile(out_path.string(), *cloud, binary_mode);
  }
  else if(extension == ".qc")
  {
    std::cout << "QC export not implemented yet" << std::endl;
    exit(1);
  }
  else
  {
    std::cerr << "Output pointcloud type " << extension << " is not supported";
  }
}

int main(int argc, char ** argv)
{
  po::variables_map vm;
  po::options_description desc("Allowed Options");

  // clang-format off
  desc.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(), "Input mesh (supported by ASSIMP)")
    ("out", po::value<std::string>(), "Output file (ply, pcd, qc)")
    ("samples", po::value<unsigned>()->default_value(10000), "Number of points to sample")
    ("type", po::value<std::string>()->default_value("xyz_rgb_normal"), "Type of cloud to generate (xyz, xyz_rgb, xyz_rgb_normal)")
    ("binary", po::bool_switch()->default_value(false), "Outputs pointcloud in binary format (default: false)");
  // clang-format on

  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);

  // parse arguments and save them in the variable map (vm)
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);

  if(!vm.count("in") || !vm.count("out") || vm.count("help"))
  {
    std::cout << "mesh_sampling is a command-line tool to sample pointcloud from CAD meshes\n\n";
    std::cout << "Usage: mesh_sampling [options] in out\n\n";
    std::cout << desc << "\n";
    return !vm.count("help");
  }

  std::string in_path = vm["in"].as<std::string>();
  std::string out_path = vm["out"].as<std::string>();
  bfs::path in_p(in_path);
  bfs::path out_p(out_path);
  unsigned N = vm["samples"].as<unsigned>();
  std::string cloud_type = vm["type"].as<std::string>();
  bool cloud_binary = vm["binary"].as<bool>();
  auto supported_cloud_type = std::vector<std::string>{"xyz", "xyz_rgb", "xyz_normal", "xyz_rgb_normal"};
  auto supported_extensions = std::vector<std::string>{".ply", ".pcd", ".qc"};

  auto check_supported = [](const std::vector<std::string> & supported, const std::string & value)
  {
    if(std::find(supported.begin(), supported.end(), value) == supported.end())
    {
      std::cerr << "This program only supports: ";
      std::copy(supported.begin(), supported.end(), std::ostream_iterator<std::string>(std::cerr, ", "));
      std::cerr << std::endl;
      exit(1);
    }
  };
  check_supported(supported_cloud_type, cloud_type);
  check_supported(supported_extensions, out_p.extension().string());

  // Generate pointcloud
  // ASSIMPScene loader should be used and kept in scope for as long as the mesh
  // is needed.
  // The default constructor loads the mesh with all the required
  // post-processing according to point type (normal computation...)
  std::unique_ptr<ASSIMPScene> mesh = nullptr;
  try
  {
    mesh = std::unique_ptr<ASSIMPScene>(new ASSIMPScene(in_path));
  }
  catch(std::runtime_error & e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "Sampling " << N << " points from " << in_path << std::endl;
  if(cloud_type == "xyz")
  {
    create_cloud<pcl::PointXYZ>(mesh->scene(), N, out_p, cloud_binary);
    // WeightedRandomSampling<pcl::PointXYZ> sampling_xyz(mesh->scene());
    // auto cloud_xyz = sampling_xyz.weighted_random_sampling(N);
    // pcl::io::savePCDFileASCII("/tmp/example_xyz.pcd", *cloud_xyz);
    // std::cout << "Cloud size: " << cloud_xyz->size() << ", expected: " << N << std::endl;
  }
  else if(cloud_type == "xyz_rgb")
  {
    create_cloud<pcl::PointXYZRGB>(mesh->scene(), N, out_p, cloud_binary);
    // WeightedRandomSampling<pcl::PointXYZRGB> sampling_rgb(mesh->scene());
    // auto cloud_rgb = sampling_rgb.weighted_random_sampling(N);
    // pcl::io::savePCDFileASCII("/tmp/example_rgb.pcd", *cloud_rgb);
  }
  else if(cloud_type == "xyz_normal")
  {
    create_cloud<pcl::PointXYZRGB>(mesh->scene(), N, out_p, cloud_binary);
    // WeightedRandomSampling<pcl::PointNormal> sampling_normal(mesh->scene());
    // auto cloud_normal = sampling_normal.weighted_random_sampling(N);
    // pcl::io::savePCDFileASCII("/tmp/example_normal.pcd", *cloud_normal);
  }
  else if(cloud_type == "xyz_rgb_normal")
  {
    create_cloud<pcl::PointXYZRGBNormal>(mesh->scene(), N, out_p, cloud_binary);
    // WeightedRandomSampling<pcl::PointXYZRGBNormal> sampling_rgb_normal(mesh->scene());
    // auto cloud_rgb_normal = sampling_rgb_normal.weighted_random_sampling(N);
    // pcl::io::savePCDFileASCII("/tmp/example_rgb_normal.pcd", *cloud_rgb_normal);
  }

  return 0;
}
