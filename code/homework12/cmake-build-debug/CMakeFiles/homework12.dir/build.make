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
CMAKE_SOURCE_DIR = /home/smartuil/SLAM-FromZeroToHero/code/homework12

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/homework12.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/homework12.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/homework12.dir/flags.make

CMakeFiles/homework12.dir/virtual-billboard.cpp.o: CMakeFiles/homework12.dir/flags.make
CMakeFiles/homework12.dir/virtual-billboard.cpp.o: ../virtual-billboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/homework12.dir/virtual-billboard.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/homework12.dir/virtual-billboard.cpp.o -c /home/smartuil/SLAM-FromZeroToHero/code/homework12/virtual-billboard.cpp

CMakeFiles/homework12.dir/virtual-billboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/homework12.dir/virtual-billboard.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/smartuil/SLAM-FromZeroToHero/code/homework12/virtual-billboard.cpp > CMakeFiles/homework12.dir/virtual-billboard.cpp.i

CMakeFiles/homework12.dir/virtual-billboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/homework12.dir/virtual-billboard.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/smartuil/SLAM-FromZeroToHero/code/homework12/virtual-billboard.cpp -o CMakeFiles/homework12.dir/virtual-billboard.cpp.s

# Object files for target homework12
homework12_OBJECTS = \
"CMakeFiles/homework12.dir/virtual-billboard.cpp.o"

# External object files for target homework12
homework12_EXTERNAL_OBJECTS =

homework12: CMakeFiles/homework12.dir/virtual-billboard.cpp.o
homework12: CMakeFiles/homework12.dir/build.make
homework12: /usr/local/lib/libopencv_dnn.so.4.2.0
homework12: /usr/local/lib/libopencv_gapi.so.4.2.0
homework12: /usr/local/lib/libopencv_highgui.so.4.2.0
homework12: /usr/local/lib/libopencv_ml.so.4.2.0
homework12: /usr/local/lib/libopencv_objdetect.so.4.2.0
homework12: /usr/local/lib/libopencv_photo.so.4.2.0
homework12: /usr/local/lib/libopencv_stitching.so.4.2.0
homework12: /usr/local/lib/libopencv_video.so.4.2.0
homework12: /usr/local/lib/libopencv_videoio.so.4.2.0
homework12: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
homework12: /usr/local/lib/libopencv_calib3d.so.4.2.0
homework12: /usr/local/lib/libopencv_features2d.so.4.2.0
homework12: /usr/local/lib/libopencv_flann.so.4.2.0
homework12: /usr/local/lib/libopencv_imgproc.so.4.2.0
homework12: /usr/local/lib/libopencv_core.so.4.2.0
homework12: CMakeFiles/homework12.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable homework12"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homework12.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/homework12.dir/build: homework12

.PHONY : CMakeFiles/homework12.dir/build

CMakeFiles/homework12.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/homework12.dir/cmake_clean.cmake
.PHONY : CMakeFiles/homework12.dir/clean

CMakeFiles/homework12.dir/depend:
	cd /home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/smartuil/SLAM-FromZeroToHero/code/homework12 /home/smartuil/SLAM-FromZeroToHero/code/homework12 /home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homework12/cmake-build-debug/CMakeFiles/homework12.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/homework12.dir/depend
