cmake_minimum_required (VERSION 3.10)

include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Common/include")

set(THREAD_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Communications/test/thread_test.cpp)
set(THREAD_TEST_LIBS Communications pthread)

add_executable(thread_test ${THREAD_TEST_SOURCES})
target_link_libraries(thread_test ${THREAD_TEST_LIBS} )

