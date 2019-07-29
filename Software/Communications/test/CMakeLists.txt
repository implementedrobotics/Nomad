include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Communications/include")

#set(CONTROLLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/qp_solver_test.cpp)
#set(CONTROLLER_TEST_LIBS qpOASES OptimalControl)

set(ZCM_SOURCES ${PROJECT_SOURCE_DIR}/Communications/test/zcm_hello_world.cpp)
set(ZCM_TEST_LIBS zcm pthread)


add_executable(zcm_hello_world ${PROJECT_SOURCE_DIR}/Communications/test/zcm_hello_world.cpp)
target_link_libraries(zcm_hello_world ${ZCM_TEST_LIBS} )


# add_executable(zmq_sub_client ${PROJECT_SOURCE_DIR}/Communications/test/zmq_sub_client.cpp)
# target_link_libraries(zmq_sub_client ${ZEROMQ_TEST_LIBS} )

#add_test(QuickStartTest test_main COMMAND test_main)