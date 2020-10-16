cmake_minimum_required(VERSION 3.11)

set(PROJECT_NAME test_mesh_sampling)
set(PROJECT_DESCRIPTION "Sample pointcloud from meshes")
set(PROJECT_URL https://github.com/arntanguy/mesh_sampling)
set(PROJECT_VERSION 1.0.0)
set(PROJECT_USE_CMAKE_EXPORT FALSE)
set(PROJECT_DEBUG_POSTFIX "_d")
set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})
find_package(mesh_sampling REQUIRED)

include_directories(include)
add_subdirectory(src)

