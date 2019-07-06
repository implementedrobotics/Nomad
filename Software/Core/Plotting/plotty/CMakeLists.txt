    
cmake_minimum_required(VERSION 3.10)
project(plotty)

add_definitions(-std=c++11 -Wno-sign-compare -Wno-unused-value)

# The matplotlib-cpp headers are linked against
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include")

set(PLOTTY_SOURCES ${PROJECT_SOURCE_DIR}/src/matplotlibcpp.cpp)

add_library(plotty STATIC ${PLOTTY_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARY})


