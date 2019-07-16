cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(nomad_world SHARED src/nomad_world.cpp)
target_link_libraries(nomad_world ${GAZEBO_LIBRARIES})

add_library(nomad_model SHARED src/nomad_model.cpp)
target_link_libraries(nomad_model ${GAZEBO_LIBRARIES})

 