
cmake_minimum_required (VERSION 3.10)

set(OPERATOR_INTERFACE_SOURCES ${PROJECT_SOURCE_DIR}/OperatorInterface/src/RemoteTeleop.cpp
)

set(OPERATOR_INTERFACE_LIBS Controllers pthread rt zmq)

include_directories("${PROJECT_SOURCE_DIR}/OperatorInterface/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/Controllers/include")

add_library(OperatorInterface STATIC ${OPERATOR_INTERFACE_SOURCES})
target_link_libraries(OperatorInterface ${OPERATOR_INTERFACE_LIBS})

# TODO: Make Options "Build Test/Examples etc"
#add_subdirectory(test)
