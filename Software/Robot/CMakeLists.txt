
cmake_minimum_required (VERSION 3.10)

set(ROBOT_SOURCES ${PROJECT_SOURCE_DIR}/Robot/Nomad/src/NomadRobot.cpp
)

set(ROBOT_LIBS dart dart-utils-urdf)

include_directories("${PROJECT_SOURCE_DIR}/Robot/Nomad/include")

add_library(Robot STATIC ${ROBOT_SOURCES})
target_link_libraries(Robot ${ROBOT_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)

# TODO: Make Options "Build Test/Examples etc"
#add_subdirectory(Nomad/test)
