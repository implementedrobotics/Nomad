
#include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")

# RBDL TEST
set(PHYSICS_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Core/Physics/test/rbdl_test.cpp)
set(PHYSICS_TEST_LIBS rbdl)

add_executable(rbdl_test ${PHYSICS_TEST_SOURCES})
target_link_libraries(rbdl_test ${PHYSICS_TEST_LIBS} )



# DART Tests
set(required_components)
set(required_libraries dart)
#find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)

#file(GLOB srcs "*.cpp" "*.hpp")
add_executable(dart_test dart_test.cpp)
target_link_libraries(dart_test PUBLIC ${required_libraries})

