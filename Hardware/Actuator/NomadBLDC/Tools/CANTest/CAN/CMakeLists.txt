cmake_minimum_required (VERSION 3.10)

set(CAN_SOURCES ${PROJECT_SOURCE_DIR}/CAN/src/CANDevice.cpp
)

#set(COMMON_LIBS Realtime pthread rt)


include_directories("${PROJECT_SOURCE_DIR}/CAN/include")

add_library(CAN STATIC ${CAN_SOURCES})
#target_link_libraries(CAN ${COMMON_LIBS})

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
