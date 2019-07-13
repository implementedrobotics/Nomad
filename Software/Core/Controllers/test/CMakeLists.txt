include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")

set(CONTROLLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/qp_solver_test.cpp)
set(CONTROLLER_TEST_LIBS qpOASES OptimalControl)

set(ZEROMQ_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/zero_mq_test.cpp)
set(ZEROMQ_TEST_LIBS zmq pthread)

set(TASK_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/task_node_test.cpp)
set(TASK_TEST_LIBS OptimalControl pthread rt)

add_executable(qp_solver_test ${CONTROLLER_TEST_SOURCES})
target_link_libraries(qp_solver_test ${CONTROLLER_TEST_LIBS} )

add_executable(zmq_test ${ZEROMQ_TEST_SOURCES})
target_link_libraries(zmq_test ${ZEROMQ_TEST_LIBS} )

add_executable(thread_task ${TASK_TEST_SOURCES})
target_link_libraries(thread_task ${TASK_TEST_LIBS} )

#add_test(QuickStartTest test_main COMMAND test_main)