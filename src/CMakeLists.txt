add_library(mesh_sampling SHARED
  weighted_random_sampling.cpp
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

add_executable(example example.cpp)
target_link_libraries(example mesh_sampling)
