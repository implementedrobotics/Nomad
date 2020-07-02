
cmake_minimum_required (VERSION 3.10)

set(ROBOT_SOURCES ${PROJECT_SOURCE_DIR}/Robot/Nomad/src/NomadControlFSM.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/src/NomadRobot.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/src/NomadControl.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/src/NomadDynamics.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/src/OffState.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/src/IdleState.cpp
${PROJECT_SOURCE_DIR}/Robot/Nomad/src/StandState.cpp

)

set(ROBOT_LIBS Common Communications Realtime Controllers dart dart-utils-urdf)

include_directories("${PROJECT_SOURCE_DIR}/Robot/Nomad/include")
include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/Common/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/Controllers/include")

add_library(Robot STATIC ${ROBOT_SOURCES})
target_link_libraries(Robot ${ROBOT_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(Nomad/test)
