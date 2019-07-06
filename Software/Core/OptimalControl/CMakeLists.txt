
set(OPTIMAL_CONTROL_SOURCES ${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/ControlsLibrary.cpp 
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/OptimalControlProblem.cpp
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/LinearCondensedOCP.cpp)

include_directories("${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
add_library(OptimalControl STATIC ${OPTIMAL_CONTROL_SOURCES})

#add_executable(qp_solver_test ${TEST_SOURCES})
#target_link_libraries(qp_solver_test ${TEST_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)