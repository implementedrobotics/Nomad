
find_package(PythonLibs 2.7 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/Plotting/include")
include_directories(${PYTHON_INCLUDE_DIRS})

set(PLOT_TEST_SOURCE ${PROJECT_SOURCE_DIR}/Plotting/test/plot_test.cpp)
#set(PLOT_TEST_LIBS )

# Definitions

add_executable(plot_test ${PLOT_TEST_SOURCE})
target_link_libraries(plot_test ${PYTHON_LIBRARY})

