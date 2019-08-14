
cmake_minimum_required (VERSION 3.10)

set(REALTIME_SOURCES ${PROJECT_SOURCE_DIR}/Realtime/src/RealTimeTask.cpp)

set(REALTIME_LIBS Systems Communications pthread rt)

include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/Systems/include")

add_library(Realtime STATIC ${REALTIME_SOURCES})
target_link_libraries(Realtime ${REALTIME_LIBS})
