cmake_minimum_required (VERSION 3.10)
project(Nomad)

set(NOMAD_VERSION_MAJOR 0)
set(NOMAD_VERSION_MINOR 1)

#set(SOURCES ${PROJECT_SOURCE_DIR}/src/QuickStart.cpp)

#include_directories("${PROJECT_SOURCE_DIR}/include")
#add_library(QuickStart STATIC ${SOURCES})
#enable_testing()

# Core Library
add_subdirectory(Core/Physics)
add_subdirectory(Core/Controllers)
add_subdirectory(Core/OptimalControl)
add_subdirectory(Core/Systems)
add_subdirectory(Core/Nomad)

# Common Library
add_subdirectory(Common)

# Communications Library
add_subdirectory(Communications)

# Operator Interface
add_subdirectory(OperatorInterface)

# Realtime Library
add_subdirectory(Realtime)

# Plotting Library
add_subdirectory(Plotting)

# Robot Library
add_subdirectory(Robot)

#IF(BUILD_SIMULATION)
# Simulation(Optional)
add_subdirectory(Simulation/Dart)
add_subdirectory(Simulation/Gazebo)
#ENDIF()
