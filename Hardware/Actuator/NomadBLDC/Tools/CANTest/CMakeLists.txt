cmake_minimum_required (VERSION 3.10)
project(NOMADBLDC)

set(NOMADBLDC_VERSION_MAJOR 0)
set(NOMADBLDC_VERSION_MINOR 1)

# Communications Library
add_subdirectory(CAN)

# NomadBLDC Library
add_subdirectory(NomadBLDC)




