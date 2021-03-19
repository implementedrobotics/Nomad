cmake_minimum_required (VERSION 3.10)

set(CAN_SOURCES ${PROJECT_SOURCE_DIR}/CAN/src/CANDevice.cpp 
${PROJECT_SOURCE_DIR}/CAN/src/PCANDevice.cpp 
${PROJECT_SOURCE_DIR}/CAN/src/RealTimeTask.cpp
)

set(CAN_LIBS pthread rt)


include_directories("${PROJECT_SOURCE_DIR}/CAN/include")

add_library(CAN STATIC ${CAN_SOURCES})
target_link_libraries(CAN ${CAN_LIBS})

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
