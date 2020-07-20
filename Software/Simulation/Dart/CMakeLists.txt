#DART Tests
cmake_minimum_Required(VERSION 3.5.1)

set(NOMAD_CONTROLLER_SOURCES ${PROJECT_SOURCE_DIR}/Simulation/Dart/src/NomadRobot.cpp
${PROJECT_SOURCE_DIR}/Simulation/Dart/src/main.cpp
)

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-utils-urdf dart-gui-osg osg osgViewer zcm)

#find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)

include_directories("${PROJECT_SOURCE_DIR}/Simulation/Dart/include")
include_directories("${PROJECT_SOURCE_DIR}/Communications/include")

add_executable(nomad_simulator ${NOMAD_CONTROLLER_SOURCES})
target_link_libraries(nomad_simulator ${required_libraries})
