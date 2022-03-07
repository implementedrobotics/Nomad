cmake_minimum_required(VERSION 3.10)

# Fetch pybind from github and set subdirectory
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.9.1
)

FetchContent_GetProperties(pybind11)
if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
    add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
endif()

# Includes
include_directories("${PROJECT_SOURCE_DIR}/CAN/include")
include_directories("${PROJECT_SOURCE_DIR}/NomadBLDC/include")

# Python Bindings
pybind11_add_module(nomad src/PyCANDevice.cpp)

# Library Linking
target_link_libraries(nomad PRIVATE CAN NOMADBLDC pcanfd)


