
cmake_minimum_required (VERSION 3.10)

set(PLOTTING_SOURCES ${PROJECT_SOURCE_DIR}/Plotting/src/PlotterTaskNode.cpp)

set(PLOTTING_LIBS Realtime pthread rt zcm)

find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/Plotting/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/Systems/include")

add_library(Plotting STATIC ${PLOTTING_SOURCES})
target_link_libraries(Plotting ${PLOTTING_LIBS} ${PYTHON_LIBRARY})

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
