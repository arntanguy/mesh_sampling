find_package(Boost REQUIRED COMPONENTS unit_test_framework)

macro(mesh_sampling_test NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC Boost::unit_test_framework Boost::disable_autolinking ${ARGN})
  target_compile_definitions(${NAME} PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
  target_compile_definitions(${NAME} PRIVATE -DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  add_test(${NAME} ${NAME})
endmacro()

mesh_sampling_test(testWeightedRandomSampling mesh_sampling)
