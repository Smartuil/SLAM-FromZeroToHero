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
include CMakeFiles/directBA.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/directBA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/directBA.dir/flags.make

CMakeFiles/directBA.dir/directBA.cpp.o: CMakeFiles/directBA.dir/flags.make
CMakeFiles/directBA.dir/directBA.cpp.o: ../directBA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/directBA.dir/directBA.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/directBA.dir/directBA.cpp.o -c /home/smartuil/SLAM-FromZeroToHero/code/homework18/directBA.cpp

CMakeFiles/directBA.dir/directBA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/directBA.dir/directBA.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/smartuil/SLAM-FromZeroToHero/code/homework18/directBA.cpp > CMakeFiles/directBA.dir/directBA.cpp.i

CMakeFiles/directBA.dir/directBA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/directBA.dir/directBA.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/smartuil/SLAM-FromZeroToHero/code/homework18/directBA.cpp -o CMakeFiles/directBA.dir/directBA.cpp.s

# Object files for target directBA
directBA_OBJECTS = \
"CMakeFiles/directBA.dir/directBA.cpp.o"

# External object files for target directBA
directBA_EXTERNAL_OBJECTS =

directBA: CMakeFiles/directBA.dir/directBA.cpp.o
directBA: CMakeFiles/directBA.dir/build.make
directBA: /usr/local/lib/libopencv_dnn.so.4.2.0
directBA: /usr/local/lib/libopencv_gapi.so.4.2.0
directBA: /usr/local/lib/libopencv_highgui.so.4.2.0
directBA: /usr/local/lib/libopencv_ml.so.4.2.0
directBA: /usr/local/lib/libopencv_objdetect.so.4.2.0
directBA: /usr/local/lib/libopencv_photo.so.4.2.0
directBA: /usr/local/lib/libopencv_stitching.so.4.2.0
directBA: /usr/local/lib/libopencv_video.so.4.2.0
directBA: /usr/local/lib/libopencv_videoio.so.4.2.0
directBA: /usr/local/lib/libg2o_cli.so
directBA: /usr/local/lib/libg2o_core.so
directBA: /usr/local/lib/libg2o_csparse_extension.so
directBA: /usr/local/lib/libg2o_ext_freeglut_minimal.so
directBA: /usr/local/lib/libg2o_hierarchical.so
directBA: /usr/local/lib/libg2o_incremental.so
directBA: /usr/local/lib/libg2o_interactive.so
directBA: /usr/local/lib/libg2o_interface.so
directBA: /usr/local/lib/libg2o_opengl_helper.so
directBA: /usr/local/lib/libg2o_parser.so
directBA: /usr/local/lib/libg2o_simulator.so
directBA: /usr/local/lib/libg2o_solver_cholmod.so
directBA: /usr/local/lib/libg2o_solver_csparse.so
directBA: /usr/local/lib/libg2o_solver_dense.so
directBA: /usr/local/lib/libg2o_solver_eigen.so
directBA: /usr/local/lib/libg2o_solver_pcg.so
directBA: /usr/local/lib/libg2o_solver_slam2d_linear.so
directBA: /usr/local/lib/libg2o_solver_structure_only.so
directBA: /usr/local/lib/libg2o_stuff.so
directBA: /usr/local/lib/libg2o_types_data.so
directBA: /usr/local/lib/libg2o_types_icp.so
directBA: /usr/local/lib/libg2o_types_sba.so
directBA: /usr/local/lib/libg2o_types_sclam2d.so
directBA: /usr/local/lib/libg2o_types_sim3.so
directBA: /usr/local/lib/libg2o_types_slam2d.so
directBA: /usr/local/lib/libg2o_types_slam2d_addons.so
directBA: /usr/local/lib/libg2o_types_slam3d.so
directBA: /usr/local/lib/libg2o_types_slam3d_addons.so
directBA: /usr/local/lib/libg2o_viewer.so
directBA: /usr/local/lib/libpangolin.so
directBA: /usr/local/lib/libopencv_imgcodecs.so.4.2.0
directBA: /usr/local/lib/libopencv_calib3d.so.4.2.0
directBA: /usr/local/lib/libopencv_features2d.so.4.2.0
directBA: /usr/local/lib/libopencv_flann.so.4.2.0
directBA: /usr/local/lib/libopencv_imgproc.so.4.2.0
directBA: /usr/local/lib/libopencv_core.so.4.2.0
directBA: /usr/lib/x86_64-linux-gnu/libGL.so
directBA: /usr/lib/x86_64-linux-gnu/libGLU.so
directBA: /usr/lib/x86_64-linux-gnu/libGLEW.so
directBA: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
directBA: /usr/lib/x86_64-linux-gnu/libavcodec.so
directBA: /usr/lib/x86_64-linux-gnu/libavformat.so
directBA: /usr/lib/x86_64-linux-gnu/libavutil.so
directBA: /usr/lib/x86_64-linux-gnu/libswscale.so
directBA: /usr/lib/libOpenNI.so
directBA: /usr/lib/libOpenNI2.so
directBA: /usr/lib/x86_64-linux-gnu/libpng.so
directBA: /usr/lib/x86_64-linux-gnu/libz.so
directBA: /usr/lib/x86_64-linux-gnu/libjpeg.so
directBA: /usr/lib/x86_64-linux-gnu/libtiff.so
directBA: CMakeFiles/directBA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable directBA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/directBA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/directBA.dir/build: directBA

.PHONY : CMakeFiles/directBA.dir/build

CMakeFiles/directBA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/directBA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/directBA.dir/clean

CMakeFiles/directBA.dir/depend:
	cd /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/smartuil/SLAM-FromZeroToHero/code/homework18 /home/smartuil/SLAM-FromZeroToHero/code/homework18 /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug /home/smartuil/SLAM-FromZeroToHero/code/homework18/cmake-build-debug/CMakeFiles/directBA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/directBA.dir/depend
