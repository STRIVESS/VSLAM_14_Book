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
CMAKE_SOURCE_DIR = /home/kevin/Project_code/slambook2/ch3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/Project_code/slambook2/ch3/build

# Include any dependencies generated for this target.
include useGeometry/CMakeFiles/eigenGeometry.dir/depend.make

# Include the progress variables for this target.
include useGeometry/CMakeFiles/eigenGeometry.dir/progress.make

# Include the compile flags for this target's objects.
include useGeometry/CMakeFiles/eigenGeometry.dir/flags.make

useGeometry/CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.o: useGeometry/CMakeFiles/eigenGeometry.dir/flags.make
useGeometry/CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.o: ../useGeometry/eigenGeometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/Project_code/slambook2/ch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object useGeometry/CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.o"
	cd /home/kevin/Project_code/slambook2/ch3/build/useGeometry && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.o -c /home/kevin/Project_code/slambook2/ch3/useGeometry/eigenGeometry.cpp

useGeometry/CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.i"
	cd /home/kevin/Project_code/slambook2/ch3/build/useGeometry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/Project_code/slambook2/ch3/useGeometry/eigenGeometry.cpp > CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.i

useGeometry/CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.s"
	cd /home/kevin/Project_code/slambook2/ch3/build/useGeometry && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/Project_code/slambook2/ch3/useGeometry/eigenGeometry.cpp -o CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.s

# Object files for target eigenGeometry
eigenGeometry_OBJECTS = \
"CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.o"

# External object files for target eigenGeometry
eigenGeometry_EXTERNAL_OBJECTS =

useGeometry/eigenGeometry: useGeometry/CMakeFiles/eigenGeometry.dir/eigenGeometry.cpp.o
useGeometry/eigenGeometry: useGeometry/CMakeFiles/eigenGeometry.dir/build.make
useGeometry/eigenGeometry: useGeometry/CMakeFiles/eigenGeometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/Project_code/slambook2/ch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable eigenGeometry"
	cd /home/kevin/Project_code/slambook2/ch3/build/useGeometry && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eigenGeometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
useGeometry/CMakeFiles/eigenGeometry.dir/build: useGeometry/eigenGeometry

.PHONY : useGeometry/CMakeFiles/eigenGeometry.dir/build

useGeometry/CMakeFiles/eigenGeometry.dir/clean:
	cd /home/kevin/Project_code/slambook2/ch3/build/useGeometry && $(CMAKE_COMMAND) -P CMakeFiles/eigenGeometry.dir/cmake_clean.cmake
.PHONY : useGeometry/CMakeFiles/eigenGeometry.dir/clean

useGeometry/CMakeFiles/eigenGeometry.dir/depend:
	cd /home/kevin/Project_code/slambook2/ch3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/Project_code/slambook2/ch3 /home/kevin/Project_code/slambook2/ch3/useGeometry /home/kevin/Project_code/slambook2/ch3/build /home/kevin/Project_code/slambook2/ch3/build/useGeometry /home/kevin/Project_code/slambook2/ch3/build/useGeometry/CMakeFiles/eigenGeometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : useGeometry/CMakeFiles/eigenGeometry.dir/depend

