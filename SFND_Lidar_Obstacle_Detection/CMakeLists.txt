cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#add_definitions(-std=c++14)

#set(CXX_FLAGS "-Wall -march=native")
#set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14 -Wall -march=native")

project(playback)

find_package(PCL 1.10.1 REQUIRED)

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




