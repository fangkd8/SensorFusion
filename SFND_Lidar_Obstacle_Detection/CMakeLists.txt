cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL REQUIRED QUIET)

include_directories(
    ${PCL_INCLUDE_DIRS}
    "/usr/local/include/eigen3"    
)
link_directories(
    ${PCL_LIBRARY_DIRS}
    "/usr/local/include/eigen3" 
)
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})




