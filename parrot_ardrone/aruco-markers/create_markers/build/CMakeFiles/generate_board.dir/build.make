# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build

# Include any dependencies generated for this target.
include CMakeFiles/generate_board.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/generate_board.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/generate_board.dir/flags.make

CMakeFiles/generate_board.dir/src/create_board.o: CMakeFiles/generate_board.dir/flags.make
CMakeFiles/generate_board.dir/src/create_board.o: ../src/create_board.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/generate_board.dir/src/create_board.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/generate_board.dir/src/create_board.o -c /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/src/create_board.cpp

CMakeFiles/generate_board.dir/src/create_board.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/generate_board.dir/src/create_board.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/src/create_board.cpp > CMakeFiles/generate_board.dir/src/create_board.i

CMakeFiles/generate_board.dir/src/create_board.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/generate_board.dir/src/create_board.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/src/create_board.cpp -o CMakeFiles/generate_board.dir/src/create_board.s

CMakeFiles/generate_board.dir/src/create_board.o.requires:

.PHONY : CMakeFiles/generate_board.dir/src/create_board.o.requires

CMakeFiles/generate_board.dir/src/create_board.o.provides: CMakeFiles/generate_board.dir/src/create_board.o.requires
	$(MAKE) -f CMakeFiles/generate_board.dir/build.make CMakeFiles/generate_board.dir/src/create_board.o.provides.build
.PHONY : CMakeFiles/generate_board.dir/src/create_board.o.provides

CMakeFiles/generate_board.dir/src/create_board.o.provides.build: CMakeFiles/generate_board.dir/src/create_board.o


# Object files for target generate_board
generate_board_OBJECTS = \
"CMakeFiles/generate_board.dir/src/create_board.o"

# External object files for target generate_board
generate_board_EXTERNAL_OBJECTS =

generate_board: CMakeFiles/generate_board.dir/src/create_board.o
generate_board: CMakeFiles/generate_board.dir/build.make
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
generate_board: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
generate_board: CMakeFiles/generate_board.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable generate_board"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/generate_board.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/generate_board.dir/build: generate_board

.PHONY : CMakeFiles/generate_board.dir/build

CMakeFiles/generate_board.dir/requires: CMakeFiles/generate_board.dir/src/create_board.o.requires

.PHONY : CMakeFiles/generate_board.dir/requires

CMakeFiles/generate_board.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/generate_board.dir/cmake_clean.cmake
.PHONY : CMakeFiles/generate_board.dir/clean

CMakeFiles/generate_board.dir/depend:
	cd /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build /home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build/CMakeFiles/generate_board.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/generate_board.dir/depend
