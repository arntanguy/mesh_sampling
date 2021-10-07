#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find the ASSIMP library
#
# If the library is found, then you can use the mesh_sampling::assimp target
#

if(NOT TARGET mesh_sampling::assimp)
  message(STATUS "Checking for module 'ASSIMP'")
  find_package(assimp QUIET)
  if(NOT ${assimp_FOUND})
    message(FATAL_ERROR "Could not find the ASSIMP library")
  endif()

  message(STATUS "Found assimp library: ${assimp_VERSION_MAJOR}.${assimp_VERSION_MINOR}.${assimp_VERSION_PATCH}")
  add_library(mesh_sampling::assimp INTERFACE IMPORTED)
  set_target_properties(mesh_sampling::assimp PROPERTIES
    #INTERFACE_INCLUDE_DIRECTORIES ${ASSIMP_INCLUDE_DIRS}
    INTERFACE_LINK_LIBRARIES "${ASSIMP_LIBRARIES}"
  )
endif()
