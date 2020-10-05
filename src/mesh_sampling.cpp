/*
 * Copyright 2017-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <boost/filesystem.hpp>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/qhull_io.h>
#include <mesh_sampling/weighted_random_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
namespace bfs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace mesh_sampling;

template<typename PointT>
void create_cloud(const aiScene * scene, unsigned N, const bfs::path & out_path, bool binary_mode = false)
{
  WeightedRandomSampling<PointT> sampler(scene);
  auto cloud = sampler.weighted_random_sampling(N);

  auto out = out_path.string();
  auto extension = out_path.extension();
  bool success = true;
  if(extension == ".pcd")
  {
    success = pcl::io::savePCDFile(out, *cloud, binary_mode) == 0;
  }
  else if(extension == ".ply")
  {
    success = pcl::io::savePLYFile(out, *cloud, binary_mode) == 0;
  }
  else if(extension == ".qc")
  {
    success = mesh_sampling::io::saveQhullFile(out, *cloud);
  }
  else
  {
    std::cerr << "Output pointcloud type " << extension << " is not supported";
    exit(1);
  }

  if(!success)
  {
    std::cerr << "Saving to " << out << " failed." << std::endl;
    exit(1);
  }
  else
  {
    std::cout << "Pointcloud saved to " << out << std::endl;
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

  auto check_supported = [](const std::vector<std::string> & supported, const std::string & value) {
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
  }
  else if(cloud_type == "xyz_rgb")
  {
    create_cloud<pcl::PointXYZRGB>(mesh->scene(), N, out_p, cloud_binary);
  }
  else if(cloud_type == "xyz_normal")
  {
    create_cloud<pcl::PointXYZRGB>(mesh->scene(), N, out_p, cloud_binary);
  }
  else if(cloud_type == "xyz_rgb_normal")
  {
    create_cloud<pcl::PointXYZRGBNormal>(mesh->scene(), N, out_p, cloud_binary);
  }

  return 0;
}
