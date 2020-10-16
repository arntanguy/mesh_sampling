add_executable(test_mesh_sampling main.cpp)
target_link_libraries(test_mesh_sampling PUBLIC mesh_sampling::mesh_sampling)
target_include_directories(test_mesh_sampling PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
