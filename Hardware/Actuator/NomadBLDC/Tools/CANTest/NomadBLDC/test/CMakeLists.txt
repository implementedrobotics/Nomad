cmake_minimum_required (VERSION 3.10)

include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/CAN/include")

set(NOMADBLDC_TEST_SOURCES ${PROJECT_SOURCE_DIR}/NomadBLDC/test/nomadbldc_test.cpp)
set(NOMADBLDC_TEST_LIBS CAN NOMADBLDC pcanfd zmq)

set(TELEPRESENCE_TEST_SOURCES ${PROJECT_SOURCE_DIR}/NomadBLDC/test/telepresence_test.cpp)
set(TELEPRESENCE_TEST_LIBS CAN NOMADBLDC pcanfd zmq)

set(PLOTJUGGLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/NomadBLDC/test/plotjuggler_test.cpp)
set(PLOTJUGGLER_TEST_LIBS CAN NOMADBLDC pcanfd zmq)

# Definitions
#add_definitions(-D_GNU_SOURCE)

add_executable(nomadbldc_test ${NOMADBLDC_TEST_SOURCES})
target_link_libraries(nomadbldc_test ${NOMADBLDC_TEST_LIBS} )

add_executable(telepresence_test ${TELEPRESENCE_TEST_SOURCES})
target_link_libraries(telepresence_test ${TELEPRESENCE_TEST_LIBS} )

add_executable(plotjuggler_test ${PLOTJUGGLER_TEST_SOURCES})
target_link_libraries(plotjuggler_test ${PLOTJUGGLER_TEST_LIBS} )
