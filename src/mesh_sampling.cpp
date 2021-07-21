/*
 * Copyright 2017-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/qhull_io.h>
#include <mesh_sampling/weighted_random_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
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
  auto extension = boost::algorithm::to_lower_copy(out_path.extension().string());
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
  else if(extension == ".stl")
  {
    pcl::ConvexHull<PointT> convex{};
    const auto shared_cloud = boost::shared_ptr<const pcl::PointCloud<PointT>>(cloud.release());
    convex.setInputCloud(shared_cloud);
    pcl::PolygonMesh mesh;
    convex.reconstruct(mesh);
    pcl::PointCloud<PointT> cloud{};
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    // Compute the convex center
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for(auto & poly : mesh.polygons)
    {
      if(poly.vertices.size() != 3)
      {
        throw std::runtime_error("pcl::ConvexHull did not reconstruct a triangular mesh");
      }
      center += (cloud[poly.vertices[0]].getVector3fMap() + cloud[poly.vertices[1]].getVector3fMap()
                 + cloud[poly.vertices[2]].getVector3fMap())
                / (3 * mesh.polygons.size());
    }

    // Make sure all normals point away from the mesh center
    // Since the mesh is convex this is enough to ensure the normals are consistent
    for(auto & poly : mesh.polygons)
    {
      Eigen::Vector3f p1 = cloud[poly.vertices[0]].getVector3fMap();
      Eigen::Vector3f p2 = cloud[poly.vertices[1]].getVector3fMap();
      Eigen::Vector3f p3 = cloud[poly.vertices[2]].getVector3fMap();
      Eigen::Vector3f n = (p2 - p1).cross(p3 - p1);
      if(n.dot(center - p1) > 0)
      {
        std::swap(poly.vertices[1], poly.vertices[2]);
      }
    }
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
    success = pcl::io::savePolygonFileSTL(out, mesh, binary_mode);
#else
    success = pcl::io::savePolygonFileSTL(out, mesh);
#endif
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
}

int main(int argc, char ** argv)
{
  po::variables_map vm;
  po::options_description desc("Allowed Options");

  // clang-format off
  desc.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(), "Input mesh (supported by ASSIMP)")
    ("out", po::value<std::string>(), "Output file (ply, pcd, qc, stl)")
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
  auto out_extension = boost::algorithm::to_lower_copy(out_p.extension().string());
  unsigned N = vm["samples"].as<unsigned>();
  std::string cloud_type = vm["type"].as<std::string>();
  bool cloud_binary = vm["binary"].as<bool>();
  auto supported_cloud_type = std::vector<std::string>{"xyz", "xyz_rgb", "xyz_normal", "xyz_rgb_normal"};
  auto supported_extensions = std::vector<std::string>{".ply", ".pcd", ".qc", ".stl"};

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
  check_supported(supported_extensions, out_extension);

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
    create_cloud<pcl::PointNormal>(mesh->scene(), N, out_p, cloud_binary);
  }
  else if(cloud_type == "xyz_rgb_normal")
  {
    create_cloud<pcl::PointXYZRGBNormal>(mesh->scene(), N, out_p, cloud_binary);
  }

  return 0;
}
