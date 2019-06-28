
#include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")

set(PHYSICS_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Physics/test/rbdl_test.cpp)
set(PHYSICS_TEST_LIBS rbdl)

add_executable(rbdl_test ${PHYSICS_TEST_SOURCES})
target_link_libraries(rbdl_test ${PHYSICS_TEST_LIBS} )