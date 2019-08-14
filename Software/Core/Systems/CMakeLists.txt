
set(SYSTEMS_SOURCES ${PROJECT_SOURCE_DIR}/Core/Systems/src/DynamicalSystem.cpp
${PROJECT_SOURCE_DIR}/Core/Systems/src/RigidBody.cpp
${PROJECT_SOURCE_DIR}/Core/Systems/src/NomadPlant.cpp
${PROJECT_SOURCE_DIR}/Core/Systems/src/Time.cpp
)

#set(required_components)
set(required_libraries dart)

set(SYSTEMS_LIBS Realtime pthread rt)

include_directories("${PROJECT_SOURCE_DIR}/Core/Systems/include")
include_directories("${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
include_directories("${PROJECT_SOURCE_DIR}/Realtime/include")
include_directories("${PROJECT_SOURCE_DIR}/Communications/include")
add_library(Systems STATIC ${SYSTEMS_SOURCES})


target_link_libraries(Systems ${SYSTEMS_TEST_LIBS} PUBLIC ${required_libraries})

add_subdirectory(test)
#add_executable(rigid_body_system ${SYSTEMS_SOURCES})
#target_link_libraries(qp_solver_test ${TEST_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)
