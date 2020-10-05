add_library(mesh_sampling SHARED
  weighted_random_sampling.cpp
  ../include/mesh_sampling/assimp_scene.h
  ../include/mesh_sampling/weighted_random_sampling.h
  ../include/mesh_sampling/qhull_io.h
  )
target_link_libraries(mesh_sampling PUBLIC mesh_sampling::assimp)
target_link_libraries(mesh_sampling PUBLIC mesh_sampling::PCL)
target_include_directories(mesh_sampling PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(TARGETS mesh_sampling 
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

add_executable(mesh_sampling_bin mesh_sampling.cpp)
target_link_libraries(mesh_sampling_bin PUBLIC mesh_sampling)
target_link_libraries(mesh_sampling_bin PUBLIC Boost::program_options Boost::filesystem)
set_target_properties(mesh_sampling_bin PROPERTIES OUTPUT_NAME mesh_sampling)

install(TARGETS mesh_sampling_bin
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
