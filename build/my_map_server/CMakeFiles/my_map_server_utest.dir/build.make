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
include my_map_server/CMakeFiles/my_map_server_utest.dir/depend.make

# Include the progress variables for this target.
include my_map_server/CMakeFiles/my_map_server_utest.dir/progress.make

# Include the compile flags for this target's objects.
include my_map_server/CMakeFiles/my_map_server_utest.dir/flags.make

my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o: my_map_server/CMakeFiles/my_map_server_utest.dir/flags.make
my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o: /home/oleffa/catkin_ws/src/my_map_server/test/utest.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/oleffa/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o"
	cd /home/oleffa/catkin_ws/build/my_map_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o -c /home/oleffa/catkin_ws/src/my_map_server/test/utest.cpp

my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_map_server_utest.dir/test/utest.cpp.i"
	cd /home/oleffa/catkin_ws/build/my_map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/oleffa/catkin_ws/src/my_map_server/test/utest.cpp > CMakeFiles/my_map_server_utest.dir/test/utest.cpp.i

my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_map_server_utest.dir/test/utest.cpp.s"
	cd /home/oleffa/catkin_ws/build/my_map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/oleffa/catkin_ws/src/my_map_server/test/utest.cpp -o CMakeFiles/my_map_server_utest.dir/test/utest.cpp.s

my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.requires:
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.requires

my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.provides: my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.requires
	$(MAKE) -f my_map_server/CMakeFiles/my_map_server_utest.dir/build.make my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.provides.build
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.provides

my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.provides.build: my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o

my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o: my_map_server/CMakeFiles/my_map_server_utest.dir/flags.make
my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o: /home/oleffa/catkin_ws/src/my_map_server/test/test_constants.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/oleffa/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o"
	cd /home/oleffa/catkin_ws/build/my_map_server && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o -c /home/oleffa/catkin_ws/src/my_map_server/test/test_constants.cpp

my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.i"
	cd /home/oleffa/catkin_ws/build/my_map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/oleffa/catkin_ws/src/my_map_server/test/test_constants.cpp > CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.i

my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.s"
	cd /home/oleffa/catkin_ws/build/my_map_server && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/oleffa/catkin_ws/src/my_map_server/test/test_constants.cpp -o CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.s

my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.requires:
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.requires

my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.provides: my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.requires
	$(MAKE) -f my_map_server/CMakeFiles/my_map_server_utest.dir/build.make my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.provides.build
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.provides

my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.provides.build: my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o

# Object files for target my_map_server_utest
my_map_server_utest_OBJECTS = \
"CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o" \
"CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o"

# External object files for target my_map_server_utest
my_map_server_utest_EXTERNAL_OBJECTS =

/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o
/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o
/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: my_map_server/CMakeFiles/my_map_server_utest.dir/build.make
/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: gtest/libgtest.so
/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: /home/oleffa/catkin_ws/devel/lib/libmap_server_image_loader.so
/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest: my_map_server/CMakeFiles/my_map_server_utest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest"
	cd /home/oleffa/catkin_ws/build/my_map_server && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_map_server_utest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
my_map_server/CMakeFiles/my_map_server_utest.dir/build: /home/oleffa/catkin_ws/devel/lib/my_map_server/my_map_server_utest
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/build

my_map_server/CMakeFiles/my_map_server_utest.dir/requires: my_map_server/CMakeFiles/my_map_server_utest.dir/test/utest.cpp.o.requires
my_map_server/CMakeFiles/my_map_server_utest.dir/requires: my_map_server/CMakeFiles/my_map_server_utest.dir/test/test_constants.cpp.o.requires
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/requires

my_map_server/CMakeFiles/my_map_server_utest.dir/clean:
	cd /home/oleffa/catkin_ws/build/my_map_server && $(CMAKE_COMMAND) -P CMakeFiles/my_map_server_utest.dir/cmake_clean.cmake
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/clean

my_map_server/CMakeFiles/my_map_server_utest.dir/depend:
	cd /home/oleffa/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/oleffa/catkin_ws/src /home/oleffa/catkin_ws/src/my_map_server /home/oleffa/catkin_ws/build /home/oleffa/catkin_ws/build/my_map_server /home/oleffa/catkin_ws/build/my_map_server/CMakeFiles/my_map_server_utest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : my_map_server/CMakeFiles/my_map_server_utest.dir/depend
