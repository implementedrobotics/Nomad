include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/OperatorInterface/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Common/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Robot/Nomad/include")

set(NOMAD_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Robot/Nomad/test/nomad_test.cpp)
set(NOMAD_TEST_LIBS Realtime Communications OperatorInterface Common)

# Definitions
add_definitions(-D_GNU_SOURCE)

add_executable(nomad_test ${NOMAD_TEST_SOURCES})
target_link_libraries(nomad_test ${NOMAD_TEST_LIBS} )
