# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/smartuil/clion-2019.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/smartuil/clion-2019.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/smartuil/SLAM-FromZeroToHero/code/homework18

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/homework18.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/homework18.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/homework18.dir/flags.make

CMakeFiles/homework18.dir/main.cpp.o: CMakeFiles/homework18.dir/flags.make
CMakeFiles/homework18.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/homework18.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/homework18.dir/main.cpp.o -c /home/smartuil/SLAM-FromZeroToHero/code/homework18/main.cpp

CMakeFiles/homework18.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/homework18.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/smartuil/SLAM-FromZeroToHero/code/homework18/main.cpp > CMakeFiles/homework18.dir/main.cpp.i

CMakeFiles/homework18.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/homework18.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/smartuil/SLAM-FromZeroToHero/code/homework18/main.cpp -o CMakeFiles/homework18.dir/main.cpp.s

# Object files for target homework18
homework18_OBJECTS = \
"CMakeFiles/homework18.dir/main.cpp.o"

# External object files for target homework18
homework18_EXTERNAL_OBJECTS =

homework18: CMakeFiles/homework18.dir/main.cpp.o
homework18: CMakeFiles/homework18.dir/build.make
homework18: CMakeFiles/homework18.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable homework18"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homework18.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/homework18.dir/build: homework18

.PHONY : CMakeFiles/homework18.dir/build

CMakeFiles/homework18.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/homework18.dir/cmake_clean.cmake
.PHONY : CMakeFiles/homework18.dir/clean

CMakeFiles/homework18.dir/depend:
	cd /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/smartuil/SLAM-FromZeroToHero/code/homework18 /home/smartuil/SLAM-FromZeroToHero/code/homework18 /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug/CMakeFiles/homework18.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/homework18.dir/depend

