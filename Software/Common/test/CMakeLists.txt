cmake_minimum_required (VERSION 3.10)

include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Common/include")

set(FSM_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Common/test/fsm_test.cpp)
set(FSM_TEST_LIBS Common)

# Definitions
#add_definitions(-D_GNU_SOURCE)

add_executable(fsm_test ${FSM_TEST_SOURCES})
target_link_libraries(fsm_test ${FSM_TEST_LIBS} )

