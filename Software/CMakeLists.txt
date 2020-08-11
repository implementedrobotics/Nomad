cmake_minimum_required (VERSION 3.10)
project(Nomad)

set(NOMAD_VERSION_MAJOR 0)
set(NOMAD_VERSION_MINOR 1)

#set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_CXX_FLAGS "-O3")

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
