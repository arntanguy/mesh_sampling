add_library(mesh_sampling SHARED
  weighted_random_sampling.cpp
  )
pkg_config_use_dependency(mesh_sampling assimp)
target_link_libraries(mesh_sampling ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

install(TARGETS mesh_sampling 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

add_executable(example example.cpp)
target_link_libraries(example mesh_sampling)
