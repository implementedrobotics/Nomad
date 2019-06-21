include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")

set(CONTROLLER_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/test/qp_solver_test.cpp)
set(CONTROLLER_TEST_LIBS qpOASES OptimalControl)

add_executable(qp_solver_test ${CONTROLLER_TEST_SOURCES})
target_link_libraries(qp_solver_test ${CONTROLLER_TEST_LIBS} )
#add_test(QuickStartTest test_main COMMAND test_main)