# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/oleffa/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/oleffa/catkin_ws/build

# Include any dependencies generated for this target.
include volksbot/CMakeFiles/kbcontrol.dir/depend.make

# Include the progress variables for this target.
include volksbot/CMakeFiles/kbcontrol.dir/progress.make

# Include the compile flags for this target's objects.
include volksbot/CMakeFiles/kbcontrol.dir/flags.make

volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o: volksbot/CMakeFiles/kbcontrol.dir/flags.make
volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o: /home/oleffa/catkin_ws/src/volksbot/src/kbcontrol.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/oleffa/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o"
	cd /home/oleffa/catkin_ws/build/volksbot && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o -c /home/oleffa/catkin_ws/src/volksbot/src/kbcontrol.cc

volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.i"
	cd /home/oleffa/catkin_ws/build/volksbot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/oleffa/catkin_ws/src/volksbot/src/kbcontrol.cc > CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.i

volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.s"
	cd /home/oleffa/catkin_ws/build/volksbot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/oleffa/catkin_ws/src/volksbot/src/kbcontrol.cc -o CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.s

volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.requires:
.PHONY : volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.requires

volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.provides: volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.requires
	$(MAKE) -f volksbot/CMakeFiles/kbcontrol.dir/build.make volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.provides.build
.PHONY : volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.provides

volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.provides.build: volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o

# Object files for target kbcontrol
kbcontrol_OBJECTS = \
"CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o"

# External object files for target kbcontrol
kbcontrol_EXTERNAL_OBJECTS =

/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: volksbot/CMakeFiles/kbcontrol.dir/build.make
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libtf.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libtf2_ros.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libactionlib.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libmessage_filters.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libroscpp.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libxmlrpcpp.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libtf2.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libroscpp_serialization.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/librosconsole.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/librosconsole_log4cxx.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/librosconsole_backend_interface.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/librostime.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /opt/ros/jade/lib/libcpp_common.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol: volksbot/CMakeFiles/kbcontrol.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol"
	cd /home/oleffa/catkin_ws/build/volksbot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kbcontrol.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
volksbot/CMakeFiles/kbcontrol.dir/build: /home/oleffa/catkin_ws/devel/lib/volksbot/kbcontrol
.PHONY : volksbot/CMakeFiles/kbcontrol.dir/build

volksbot/CMakeFiles/kbcontrol.dir/requires: volksbot/CMakeFiles/kbcontrol.dir/src/kbcontrol.cc.o.requires
.PHONY : volksbot/CMakeFiles/kbcontrol.dir/requires

volksbot/CMakeFiles/kbcontrol.dir/clean:
	cd /home/oleffa/catkin_ws/build/volksbot && $(CMAKE_COMMAND) -P CMakeFiles/kbcontrol.dir/cmake_clean.cmake
.PHONY : volksbot/CMakeFiles/kbcontrol.dir/clean

volksbot/CMakeFiles/kbcontrol.dir/depend:
	cd /home/oleffa/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/oleffa/catkin_ws/src /home/oleffa/catkin_ws/src/volksbot /home/oleffa/catkin_ws/build /home/oleffa/catkin_ws/build/volksbot /home/oleffa/catkin_ws/build/volksbot/CMakeFiles/kbcontrol.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : volksbot/CMakeFiles/kbcontrol.dir/depend
