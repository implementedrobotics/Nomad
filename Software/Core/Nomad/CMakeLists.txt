include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/Controllers/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/Nomad/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/Physics/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/Systems/include")
include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Core/Simulation/include")

set(NOMAD_SOURCES ${PROJECT_SOURCE_DIR}/Core/Nomad/src/Nomad.cpp)
set(NOMAD_LIBS Systems)

add_executable(nomad_core ${NOMAD_SOURCES})
target_link_libraries(nomad_core ${NOMAD_LIBS} )
