include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/OperatorInterface/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Common/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Robot/Nomad/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Robot/Nomad/test")

set(NOMAD_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Robot/Nomad/test/nomad_test.cpp)
set(NOMAD_TEST_LIBS Realtime Communications Robot Common)

set(STAND_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Robot/Nomad/test/stand_test.cpp)
set(STAND_TEST_LIBS Realtime Communications Robot Common)

set(THREAD_TEST_SOURCES ${PROJECT_SOURCE_DIR}/Robot/Nomad/test/thread_comm_test.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/test/TaskWrite.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/test/TaskRead.cpp)
set(THREAD_TEST_LIBS Realtime Communications Robot Common)

# Definitions
#add_definitions(-D_GNU_SOURCE)

add_executable(nomad_test ${NOMAD_TEST_SOURCES})
target_link_libraries(nomad_test ${NOMAD_TEST_LIBS} )

add_executable(stand_test ${STAND_TEST_SOURCES})
target_link_libraries(stand_test ${STAND_TEST_LIBS} )

add_executable(thread_comm_test ${THREAD_TEST_SOURCES})
target_link_libraries(thread_comm_test ${THREAD_TEST_LIBS} )