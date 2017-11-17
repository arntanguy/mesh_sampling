cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/eigen.cmake)
include(cmake/boost.cmake)

set(PROJECT_NAME gram_savitzky_golay)
set(PROJECT_DESCRIPTION "Sample pointcloud from meshes")
set(PROJECT_URL "")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

setup_project()
search_for_eigen()
enable_testing()

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})

add_required_dependency(assimp)

# Enable C++11
add_definitions(-std=c++11 -O3)
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
add_subdirectory(src)

setup_project_finalize()

