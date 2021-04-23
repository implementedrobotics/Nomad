cmake_minimum_required (VERSION 3.10)

set(NOMADBLDC_SOURCES ${PROJECT_SOURCE_DIR}/NomadBLDC/src/NomadBLDC.cpp 
)

set(NOMADBLDC_LIBS CAN)


include_directories("${PROJECT_SOURCE_DIR}/CAN/include")
include_directories("${PROJECT_SOURCE_DIR}/NomadBLDC/include")
include_directories("${PROJECT_SOURCE_DIR}/ThirdParty/nlohmann/include") # JSON



add_library(NOMADBLDC STATIC ${NOMADBLDC_SOURCES})
target_link_libraries(NOMADBLDC ${NOMADBLDC_LIBS})

# TODO: Make Options "Build Test/Examples etc"
add_subdirectory(test)
