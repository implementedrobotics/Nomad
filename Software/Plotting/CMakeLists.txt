
cmake_minimum_required (VERSION 3.10)

set(PLOTTING_SOURCES ${PROJECT_SOURCE_DIR}/Plotting/src/Plotting.cpp)

set(PLOTTING_LIBS Realtime pthread rt zmq)

include_directories("${PROJECT_SOURCE_DIR}/Core/Controllers/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")

add_library(Plotting STATIC ${PLOTTING_SOURCES})
target_link_libraries(Plotting ${PLOTTING_LIBS})

# TODO: Make Options "Build Test/Examples etc"
#add_subdirectory(test)
