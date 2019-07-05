
set(SYSTEMS_SOURCES ${PROJECT_SOURCE_DIR}/Core/Systems/src/DynamicalSystem.cpp)

include_directories("${PROJECT_SOURCE_DIR}/Core/Systems/include")
add_library(Systems STATIC ${SYSTEMS_SOURCES})

add_subdirectory(test)
#add_executable(rigid_body_system ${SYSTEMS_SOURCES})
#target_link_libraries(qp_solver_test ${TEST_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)
