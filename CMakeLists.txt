cmake_minimum_required(VERSION 3.11)

set(PROJECT_NAME mesh_sampling)
set(PROJECT_DESCRIPTION "Sample pointcloud from meshes")
set(PROJECT_URL https://github.com/arntanguy/mesh_sampling)
set(PROJECT_VERSION 1.0.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

include(cmake/base.cmake)
include(cmake/eigen.cmake)
search_for_eigen()

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# Note:
# Instead of these find_package calls add_project_dependecy should be used to populate the
# mesh_sampling_DEPENDENCIES of the generated mesh_samplingConfig.cmake file
#   add_project_dependency(mesh_sampling_3rd_party_assimp REQUIRED)
#   add_project_dependency(mesh_sampling_3rd_party_PCL REQUIRED)

# However, since we are using custom cmake scripts to look for these libraries and create clean modern cmake targets, using add_project_dependency leads to the mesh_samplingConfig script to try and find these dependencies before the path to the required mesh_sampling_3rd_party_*.cmake have been included.
#
# For now, manually add those to the mesh_samplingConfig.cmake script
find_package(mesh_sampling_3rd_party_assimp REQUIRED)
find_package(mesh_sampling_3rd_party_PCL REQUIRED)

# Export scripts to find the custom PCL/assimp dependencies
install(FILES
  CMakeModules/Findmesh_sampling_3rd_party_PCL.cmake
  DESTINATION /usr/local/lib/cmake/mesh_sampling
  RENAME mesh_sampling_3rd_party_PCLConfig.cmake
)

install(FILES
  CMakeModules/Findmesh_sampling_3rd_party_assimp.cmake
  DESTINATION /usr/local/lib/cmake/mesh_sampling
  RENAME mesh_sampling_3rd_party_assimpConfig.cmake
)

# Export custom package dependencies to the mesh_samplingConfig script such that they are found
# when importing the package with find_package(mesh_sampling)
set(PACKAGE_EXTRA_MACROS
"include(\"\$\{CMAKE_CURRENT_LIST_DIR\}/mesh_sampling_3rd_party_assimpConfig.cmake\")
include(\"\$\{CMAKE_CURRENT_LIST_DIR\}/mesh_sampling_3rd_party_PCLConfig.cmake\")"
)

add_subdirectory(src)

