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
CMAKE_SOURCE_DIR = /home/smartuil/SLAM-FromZeroToHero/code/homewrok17

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/BA.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BA.dir/flags.make

CMakeFiles/BA.dir/BA-3Dto2D.cpp.o: CMakeFiles/BA.dir/flags.make
CMakeFiles/BA.dir/BA-3Dto2D.cpp.o: ../BA-3Dto2D.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BA.dir/BA-3Dto2D.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BA.dir/BA-3Dto2D.cpp.o -c /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/BA-3Dto2D.cpp

CMakeFiles/BA.dir/BA-3Dto2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BA.dir/BA-3Dto2D.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/BA-3Dto2D.cpp > CMakeFiles/BA.dir/BA-3Dto2D.cpp.i

CMakeFiles/BA.dir/BA-3Dto2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BA.dir/BA-3Dto2D.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/BA-3Dto2D.cpp -o CMakeFiles/BA.dir/BA-3Dto2D.cpp.s

# Object files for target BA
BA_OBJECTS = \
"CMakeFiles/BA.dir/BA-3Dto2D.cpp.o"

# External object files for target BA
BA_EXTERNAL_OBJECTS =

BA: CMakeFiles/BA.dir/BA-3Dto2D.cpp.o
BA: CMakeFiles/BA.dir/build.make
BA: /usr/local/lib/libopencv_dnn.so.4.2.0
BA: /usr/local/lib/libopencv_gapi.so.4.2.0
BA: /usr/local/lib/libopencv_highgui.so.4.2.0
BA: /usr/local/lib/libopencv_ml.so.4.2.0
BA: /usr/local/lib/libopencv_objdetect.so.4.2.0
BA: /usr/local/lib/libopencv_photo.so.4.2.0
BA: /usr/local/lib/libopencv_stitching.so.4.2.0
BA: /usr/local/lib/libopencv_video.so.4.2.0
BA: /usr/local/lib/libopencv_videoio.so.4.2.0
BA: /usr/lib/x86_64-linux-gnu/libcxsparse.so
BA: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
BA: /usr/local/lib/libopencv_calib3d.so.4.2.0
BA: /usr/local/lib/libopencv_features2d.so.4.2.0
BA: /usr/local/lib/libopencv_flann.so.4.2.0
BA: /usr/local/lib/libopencv_imgproc.so.4.2.0
BA: /usr/local/lib/libopencv_core.so.4.2.0
BA: CMakeFiles/BA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable BA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BA.dir/build: BA

.PHONY : CMakeFiles/BA.dir/build

CMakeFiles/BA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BA.dir/clean

CMakeFiles/BA.dir/depend:
	cd /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/smartuil/SLAM-FromZeroToHero/code/homewrok17 /home/smartuil/SLAM-FromZeroToHero/code/homewrok17 /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homewrok17/cmake-build-debug/CMakeFiles/BA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BA.dir/depend

