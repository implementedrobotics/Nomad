cmake_minimum_required (VERSION 3.10)

set(COMMON_SOURCES ${PROJECT_SOURCE_DIR}/Common/src/FiniteStateMachine.cpp
)

set(COMMON_LIBS Realtime pthread rt)


include_directories("${PROJECT_SOURCE_DIR}/Common/include")

add_library(Common STATIC ${COMMON_SOURCES})
target_link_libraries(Common ${COMMON_LIBS})

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
