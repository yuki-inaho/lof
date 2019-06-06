cmake_minimum_required(VERSION 3.5)
project(lof)

set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "-O3 -std=gnu++11 -fopenmp -fexceptions")

include_directories(
  include
)

add_executable(lof 
      src/main.cpp
      src/ParameterManager.cpp
    )
find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(lof ${PCL_LIBRARIES})
include_directories(${VTK_INCLUDE_DIRS})
target_link_libraries(lof ${VTK_LIBRARIES})

