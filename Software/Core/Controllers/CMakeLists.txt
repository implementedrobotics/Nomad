
cmake_minimum_required (VERSION 3.10)

set(CONTROLLERS_SOURCES ${PROJECT_SOURCE_DIR}/Core/Controllers/src/StateEstimator.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/ConvexMPC.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/GaitScheduler.cpp
${PROJECT_SOURCE_DIR}/Core/Controllers/src/ReferenceTrajectoryGen.cpp
)

set(CONTROLLERS_LIBS OptimalControl Systems pthread rt zcm)

include_directories("${PROJECT_SOURCE_DIR}/Core/Controllers/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/Systems/include")

add_library(Controllers STATIC ${CONTROLLERS_SOURCES})
target_link_libraries(Controllers ${CONTROLLERS_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
