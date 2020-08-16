include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/OperatorInterface/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Plotting/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Common/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Robot/Nomad/include")

set(CONTROLLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/qp_solver_test.cpp)
set(CONTROLLER_TEST_LIBS qpOASES OptimalControl)

set(ZEROMQ_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/zero_mq_test.cpp)
set(ZEROMQ_TEST_LIBS zmq pthread)

set(TASK_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/task_node_test.cpp)
set(TASK_TEST_LIBS Plotting Controllers zcm)

set(TELEOP_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/teleop_test.cpp)
set(TELEOP_TEST_LIBS Plotting Controllers Common zcm)

set(LEG_CONTROLLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/leg_controller_test.cpp)
set(LEG_CONTROLLER_TEST_LIBS Controllers Common Robot zcm)

# set(DIAGRAM_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/diagram_test.cpp)
# set(DIAGRAM_TEST_LIBS Controllers Common Communications Robot zcm)

# Definitions
add_definitions(-D_GNU_SOURCE)

add_executable(qp_solver_test ${CONTROLLER_TEST_SOURCES})
target_link_libraries(qp_solver_test ${CONTROLLER_TEST_LIBS} )

add_executable(zmq_test ${ZEROMQ_TEST_SOURCES})
target_link_libraries(zmq_test ${ZEROMQ_TEST_LIBS} )

add_executable(thread_task ${TASK_TEST_SOURCES})
target_link_libraries(thread_task ${TASK_TEST_LIBS} )

add_executable(teleop_test ${TELEOP_TEST_SOURCES})
target_link_libraries(teleop_test ${TELEOP_TEST_LIBS} )

add_executable(legcontroller_test ${LEG_CONTROLLER_TEST_SOURCES})
target_link_libraries(legcontroller_test ${LEG_CONTROLLER_TEST_LIBS} )

# add_executable(diagram_test ${DIAGRAM_TEST_SOURCES})
# target_link_libraries(diagram_test ${DIAGRAM_TEST_LIBS} )