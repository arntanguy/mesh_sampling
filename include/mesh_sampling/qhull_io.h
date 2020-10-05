/*
 * Copyright 2017-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <string>
#include <fstream>
#include <pcl/point_cloud.h>

namespace mesh_sampling
{
namespace io
{

template<typename PointT>
bool saveQhullFile(const std::string & path, const pcl::PointCloud<PointT> & cloud)
{
  std::ofstream file;
  file.open(path, std::ios::out);
  if(!file.is_open())
  {
    return false;
  }

  file << "3" << std::endl;
  file << std::to_string(cloud.size()) << std::endl;
  for(const auto & point : cloud)
  {
    file << point.x << " " << point.y << " " << point.z << std::endl;
  }

  return true;
}


} /* io */
} /* mesh_sampling */
