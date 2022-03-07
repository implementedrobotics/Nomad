# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.16.3)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(pybind11-populate NONE)

include(ExternalProject)
ExternalProject_Add(pybind11-populate
                     "UPDATE_DISCONNECTED" "False" "GIT_REPOSITORY" "https://github.com/pybind/pybind11" "GIT_TAG" "v2.9.1"
                    SOURCE_DIR          "/home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/_deps/pybind11-src"
                    BINARY_DIR          "/home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/_deps/pybind11-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)
