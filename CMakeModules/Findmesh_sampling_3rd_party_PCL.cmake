#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find the GEOS library
#
# In order:
# - try to find_package(geos) (upstream package)
# - try to use geos-config to find the geos installation prefix
# - try to find the geos library
#
# If the library is found, then you can use the PCL::PCL target
#
# This module will use GEOS_PREFIX as an hint to find either the geos-config
# executable or the geos library

if(NOT TARGET mesh_sampling::PCL)
  message(STATUS "Checking for module 'PCL >1.7 (common, io)'")
  find_package(PCL 1.7 COMPONENTS common io QUIET)
  # On ubuntu 16.04, PCL erroneously links against a non-existant vtkproj4 target
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
  if(NOT ${PCL_FOUND})
    message(FATAL_ERROR "  Could not find the PCL 1.7 library")
  else()
    message(STATUS "  Found PCL ${PCL_VERSION} (common, io)")
    add_library(mesh_sampling::PCL INTERFACE IMPORTED)
    set_target_properties(mesh_sampling::PCL PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
    )
    # If possible, we use target_link_libraries to avoid problems with link-type keywords,
    # see https://github.com/PointCloudLibrary/pcl/issues/2989
    # target_link_libraries on imported libraries is supported only since CMake 3.11
    target_link_libraries(mesh_sampling::PCL INTERFACE ${PCL_LIBRARIES})
  endif()
endif()
