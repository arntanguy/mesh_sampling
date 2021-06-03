#!/bin/bash

readonly CMAKE_BUILD_TYPE=$1
readonly this_dir=`cd $(dirname $0); pwd`
readonly root_dir=`cd $this_dir/../../../; pwd`
readonly project_dir=$root_dir/tests/test_mesh_sampling

mkdir -p $project_dir/build

cd $project_dir/build
cmake ../ -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} || exit 1
cmake --build . --config ${CMAKE_BUILD_TYPE} || exit 1
