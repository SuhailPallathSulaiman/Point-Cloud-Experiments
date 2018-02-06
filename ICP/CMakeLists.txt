cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point icp.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

add_executable (icp_cup icp_cup.cpp)
target_link_libraries (icp_cup ${PCL_LIBRARIES})

add_executable (icp_simple icp_simple.cpp)
target_link_libraries (icp_simple ${PCL_LIBRARIES})



add_executable (icp_cube icp_cube.cpp)
target_link_libraries (icp_cube ${PCL_LIBRARIES})


add_executable (icp_cup_multiple icp_cup_multiple.cpp)
target_link_libraries (icp_cup_multiple ${PCL_LIBRARIES})
