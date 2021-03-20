cmake_minimum_required (VERSION 3.10)

include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/CAN/include")

set(CAN_TEST_SOURCES ${PROJECT_SOURCE_DIR}/CAN/test/can_test.cpp)
set(CAN_TEST_LIBS CAN)


set(PCAN_TEST_SOURCES ${PROJECT_SOURCE_DIR}/CAN/test/pcan_test.cpp)
set(PCAN_TEST_LIBS CAN pcanfd)
# Definitions
#add_definitions(-D_GNU_SOURCE)

add_executable(can_test ${CAN_TEST_SOURCES})
target_link_libraries(can_test ${CAN_TEST_LIBS} )

add_executable(pcan_test ${PCAN_TEST_SOURCES})
target_link_libraries(pcan_test ${PCAN_TEST_LIBS} )
