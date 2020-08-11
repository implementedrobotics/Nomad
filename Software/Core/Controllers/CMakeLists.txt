
cmake_minimum_required (VERSION 3.10)

set(CONTROLLERS_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/src/StateEstimator.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/ConvexMPC.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/GaitScheduler.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/ReferenceTrajectoryGen.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/LegController.cpp
)

set(CONTROLLERS_LIBS OptimalControl Systems Robot pthread rt zcm dart)

include_directories("${PROJECT_SOURCE_DIR}/Core/Controllers/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/Systems/include")
include_directories("${PROJECT_SOURCE_DIR}/Common/include")
include_directories("${PROJECT_SOURCE_DIR}/Robot/Nomad/include")

#set(required_components utils-urdf gui-osg)
#set(required_libraries dart dart-gui dart-utils-urdf dart-gui-osg)
#find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)

add_library(Controllers STATIC ${CONTROLLERS_SOURCES})
target_link_libraries(Controllers ${CONTROLLERS_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
