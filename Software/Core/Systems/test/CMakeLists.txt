include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/Systems/include")

set(SYSTEMS_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Systems/test/dynamical_system_test.cpp)
set(SYSTEMS_TEST_LIBS Systems)

add_executable(dynamical_system_test ${SYSTEMS_TEST_SOURCES})
target_link_libraries(dynamical_system_test ${SYSTEMS_TEST_LIBS} )
#add_test(QuickStartTest test_main COMMAND test_main)