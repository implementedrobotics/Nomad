
cmake_minimum_required (VERSION 3.10)

set(COMMUNICATIONS_SOURCES ${PROJECT_SOURCE_DIR}/Communications/src/Port.cpp)

set(COMMUNICATIONS_LIBS zcm)

include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/Common/include")

add_library(Communications STATIC ${COMMUNICATIONS_SOURCES})
target_link_libraries(Communications ${COMMUNICATIONS_LIBS})
