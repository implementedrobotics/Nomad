# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build

# Include any dependencies generated for this target.
include CAN/CMakeFiles/CAN.dir/depend.make

# Include the progress variables for this target.
include CAN/CMakeFiles/CAN.dir/progress.make

# Include the compile flags for this target's objects.
include CAN/CMakeFiles/CAN.dir/flags.make

CAN/CMakeFiles/CAN.dir/src/CANDevice.cpp.o: CAN/CMakeFiles/CAN.dir/flags.make
CAN/CMakeFiles/CAN.dir/src/CANDevice.cpp.o: ../CAN/src/CANDevice.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CAN/CMakeFiles/CAN.dir/src/CANDevice.cpp.o"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CAN.dir/src/CANDevice.cpp.o -c /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/CANDevice.cpp

CAN/CMakeFiles/CAN.dir/src/CANDevice.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CAN.dir/src/CANDevice.cpp.i"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/CANDevice.cpp > CMakeFiles/CAN.dir/src/CANDevice.cpp.i

CAN/CMakeFiles/CAN.dir/src/CANDevice.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CAN.dir/src/CANDevice.cpp.s"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/CANDevice.cpp -o CMakeFiles/CAN.dir/src/CANDevice.cpp.s

CAN/CMakeFiles/CAN.dir/src/PCANDevice.cpp.o: CAN/CMakeFiles/CAN.dir/flags.make
CAN/CMakeFiles/CAN.dir/src/PCANDevice.cpp.o: ../CAN/src/PCANDevice.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CAN/CMakeFiles/CAN.dir/src/PCANDevice.cpp.o"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CAN.dir/src/PCANDevice.cpp.o -c /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/PCANDevice.cpp

CAN/CMakeFiles/CAN.dir/src/PCANDevice.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CAN.dir/src/PCANDevice.cpp.i"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/PCANDevice.cpp > CMakeFiles/CAN.dir/src/PCANDevice.cpp.i

CAN/CMakeFiles/CAN.dir/src/PCANDevice.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CAN.dir/src/PCANDevice.cpp.s"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/PCANDevice.cpp -o CMakeFiles/CAN.dir/src/PCANDevice.cpp.s

CAN/CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.o: CAN/CMakeFiles/CAN.dir/flags.make
CAN/CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.o: ../CAN/src/SocketCANDevice.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CAN/CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.o"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.o -c /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/SocketCANDevice.cpp

CAN/CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.i"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/SocketCANDevice.cpp > CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.i

CAN/CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.s"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN/src/SocketCANDevice.cpp -o CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.s

# Object files for target CAN
CAN_OBJECTS = \
"CMakeFiles/CAN.dir/src/CANDevice.cpp.o" \
"CMakeFiles/CAN.dir/src/PCANDevice.cpp.o" \
"CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.o"

# External object files for target CAN
CAN_EXTERNAL_OBJECTS =

CAN/libCAN.so: CAN/CMakeFiles/CAN.dir/src/CANDevice.cpp.o
CAN/libCAN.so: CAN/CMakeFiles/CAN.dir/src/PCANDevice.cpp.o
CAN/libCAN.so: CAN/CMakeFiles/CAN.dir/src/SocketCANDevice.cpp.o
CAN/libCAN.so: CAN/CMakeFiles/CAN.dir/build.make
CAN/libCAN.so: CAN/CMakeFiles/CAN.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libCAN.so"
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CAN.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CAN/CMakeFiles/CAN.dir/build: CAN/libCAN.so

.PHONY : CAN/CMakeFiles/CAN.dir/build

CAN/CMakeFiles/CAN.dir/clean:
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN && $(CMAKE_COMMAND) -P CMakeFiles/CAN.dir/cmake_clean.cmake
.PHONY : CAN/CMakeFiles/CAN.dir/clean

CAN/CMakeFiles/CAN.dir/depend:
	cd /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/CAN /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN /home/nomad/dev/Nomad/Hardware/Actuator/NomadBLDC/Drivers/build/CAN/CMakeFiles/CAN.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CAN/CMakeFiles/CAN.dir/depend

