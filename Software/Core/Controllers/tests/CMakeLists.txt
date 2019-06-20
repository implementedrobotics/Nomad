include_directories("${PROJECT_SOURCE_DIR}/include")

set(TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/tests/qp_solver_test.cpp)
set(TEST_LIBS qpOASES)

add_executable(test_main ${TEST_SOURCES})
target_link_libraries(test_main ${TEST_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)