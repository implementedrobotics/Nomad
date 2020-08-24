# Required Include Directories


# Set Compiler Sources
set(OPTIMAL_CONTROL_SOURCES ${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/ControlsLibrary.cpp 
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/OptimalControlProblem.cpp
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/LinearCondensedOCP.cpp
${PROJECT_SOURCE_DIR}/Core/OptimalControl/src/ConvexMatrixEquationSolverQP.cpp
)

include_directories("${PROJECT_SOURCE_DIR}/Core/OptimalControl/include")
include_directories("${PROJECT_SOURCE_DIR}/Common/include/")

# Set Required Libraries
set(OPTIMAL_CONTROL_LIBS Common pthread rt qpOASES)

# Crashes on large multiplies otherwise
#add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
add_definitions(-D__USE_LONG_INTEGERS__)


# Setup output
add_library(OptimalControl STATIC ${OPTIMAL_CONTROL_SOURCES})
target_link_libraries(OptimalControl ${OPTIMAL_CONTROL_LIBS})
#add_test(QuickStartTest test_main COMMAND test_main)