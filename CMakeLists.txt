cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Yaml)

#find_package (PCL 1.7.0 REQUIRED)
find_package (PCL 1.7.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Eigen3_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

add_executable (Yaml yaml.cpp)
target_link_libraries (Yaml ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

