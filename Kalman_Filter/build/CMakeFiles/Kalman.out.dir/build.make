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
CMAKE_SOURCE_DIR = /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build

# Include any dependencies generated for this target.
include CMakeFiles/Kalman.out.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Kalman.out.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Kalman.out.dir/flags.make

CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.o: CMakeFiles/Kalman.out.dir/flags.make
CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.o: ../src/Algo/Filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.o -c /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Algo/Filter.cpp

CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Algo/Filter.cpp > CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.i

CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Algo/Filter.cpp -o CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.s

CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.o: CMakeFiles/Kalman.out.dir/flags.make
CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.o: ../src/Input/Input.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.o -c /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Input/Input.cpp

CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Input/Input.cpp > CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.i

CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Input/Input.cpp -o CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.s

CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.o: CMakeFiles/Kalman.out.dir/flags.make
CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.o: ../src/Output/Output.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.o -c /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Output/Output.cpp

CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Output/Output.cpp > CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.i

CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/Output/Output.cpp -o CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.s

CMakeFiles/Kalman.out.dir/src/main.cpp.o: CMakeFiles/Kalman.out.dir/flags.make
CMakeFiles/Kalman.out.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Kalman.out.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kalman.out.dir/src/main.cpp.o -c /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/main.cpp

CMakeFiles/Kalman.out.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kalman.out.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/main.cpp > CMakeFiles/Kalman.out.dir/src/main.cpp.i

CMakeFiles/Kalman.out.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kalman.out.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/src/main.cpp -o CMakeFiles/Kalman.out.dir/src/main.cpp.s

# Object files for target Kalman.out
Kalman_out_OBJECTS = \
"CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.o" \
"CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.o" \
"CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.o" \
"CMakeFiles/Kalman.out.dir/src/main.cpp.o"

# External object files for target Kalman.out
Kalman_out_EXTERNAL_OBJECTS =

../bin/Kalman.out: CMakeFiles/Kalman.out.dir/src/Algo/Filter.cpp.o
../bin/Kalman.out: CMakeFiles/Kalman.out.dir/src/Input/Input.cpp.o
../bin/Kalman.out: CMakeFiles/Kalman.out.dir/src/Output/Output.cpp.o
../bin/Kalman.out: CMakeFiles/Kalman.out.dir/src/main.cpp.o
../bin/Kalman.out: CMakeFiles/Kalman.out.dir/build.make
../bin/Kalman.out: CMakeFiles/Kalman.out.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ../bin/Kalman.out"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Kalman.out.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Kalman.out.dir/build: ../bin/Kalman.out

.PHONY : CMakeFiles/Kalman.out.dir/build

CMakeFiles/Kalman.out.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Kalman.out.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Kalman.out.dir/clean

CMakeFiles/Kalman.out.dir/depend:
	cd /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build /home/jithu/GITHUB/Filtering-Algorithms/Kalman_Filter/build/CMakeFiles/Kalman.out.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Kalman.out.dir/depend

