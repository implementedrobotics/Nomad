# Required Include Directories
include_directories("${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")

# Set Compiler Sources
set(OPTIMAL_CONTROL_SOURCES ${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/ControlsLibrary.cpp 
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/OptimalControlProblem.cpp
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/LinearCondensedOCP.cpp
)

# Set Required Libraries
set(OPTIMAL_CONTROL_LIBS pthread rt)

# Setup output
add_library(OptimalControl STATIC ${OPTIMAL_CONTROL_SOURCES})
target_link_libraries(OptimalControl ${OPTIMAL_CONTROL_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)