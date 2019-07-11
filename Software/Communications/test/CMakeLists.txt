#include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")

#set(CONTROLLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/qp_solver_test.cpp)
#set(CONTROLLER_TEST_LIBS qpOASES OptimalControl)

set(ZEROMQ__SOURCES ${PROJECT_SOURCE_DIR}/Communications/test/zmq_pub_server.cpp)
set(ZEROMQ_TEST_LIBS zmq pthread)


add_executable(zmq_pub_server ${PROJECT_SOURCE_DIR}/Communications/test/zmq_pub_server.cpp)
target_link_libraries(zmq_pub_server ${ZEROMQ_TEST_LIBS} )


add_executable(zmq_sub_client ${PROJECT_SOURCE_DIR}/Communications/test/zmq_sub_client.cpp)
target_link_libraries(zmq_sub_client ${ZEROMQ_TEST_LIBS} )

#add_test(QuickStartTest test_main COMMAND test_main)