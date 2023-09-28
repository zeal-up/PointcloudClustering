project(ClusteringAlgo)
cmake_minimum_required(VERSION 2.8)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(ClusteringAlgo main.cpp ClusteringAlgo.hpp FastEuclideanClustering.hpp)
target_link_libraries(ClusteringAlgo ${PCL_LIBRARIES})
