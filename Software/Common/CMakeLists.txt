cmake_minimum_required (VERSION 3.10)

set(COMMON_SOURCES ${PROJECT_SOURCE_DIR}/Common/src/FiniteStateMachine.cpp
${PROJECT_SOURCE_DIR}/Common/src/State.cpp
${PROJECT_SOURCE_DIR}/Common/src/TransitionEvent.cpp
${PROJECT_SOURCE_DIR}/Common/src/Time.cpp
${PROJECT_SOURCE_DIR}/Common/src/Math/CubicPolynomialTrajectory.cpp
${PROJECT_SOURCE_DIR}/Common/src/Math/MathUtils.cpp
)

set(COMMON_LIBS Realtime pthread rt)


include_directories("${PROJECT_SOURCE_DIR}/Common/include")
include_directories("${PROJECT_SOURCE_DIR}/Common/include/Math")

add_library(Common STATIC ${COMMON_SOURCES})
target_link_libraries(Common ${COMMON_LIBS})

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
