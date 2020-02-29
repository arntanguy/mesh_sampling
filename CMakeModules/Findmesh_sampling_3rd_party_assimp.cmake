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

if(NOT TARGET mesh_sampling::assimp)
  message(STATUS "Checking for module 'ASSIMP'")
  find_package(assimp QUIET)
  if(NOT ${ASSIMP_FOUND})
    message(FATAL_ERROR "  Could not find the ASSIMP library")
  else()
    message(STATUS "  Found assimp")
    add_library(mesh_sampling::assimp INTERFACE IMPORTED)
    set_target_properties(mesh_sampling::assimp PROPERTIES
      #INTERFACE_INCLUDE_DIRECTORIES ${ASSIMP_INCLUDE_DIRS}
      INTERFACE_LINK_LIBRARIES ${ASSIMP_LIBRARIES}
    )
  endif()
endif()
