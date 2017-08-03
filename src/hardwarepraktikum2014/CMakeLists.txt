cmake_minimum_required(VERSION 2.4.6)
project(hardwarepraktikum2014)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
  tf
  volksbot
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()
catkin_package(
CATKIN_DEPENDS message_runtime volksbot
)
add_message_files(
FILES
params.msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${volksbot_INCLUDE_DIRS}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
rosbuild_add_executable(pfaderstellung src/pfaderstellung.cpp)
rosbuild_add_executable(pfadverfolgung src/pfadverfolgung.cpp src/gio_path.cpp)
rosbuild_add_executable(pfaderstellung2 src/pfaderstellung_v_2.cpp)
rosbuild_add_executable(pfaderstellung3 src/pfaderstellung_v_3.cpp)
rosbuild_add_executable(pfadverfolgung2 src/pfadverfolgung_v_2.cpp src/gio_path_v_2.cpp)
rosbuild_add_executable(pfadverfolgung3 src/pfadverfolgung_v_3.cpp src/gio_path_v_3.cpp)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

target_link_libraries(pfadverfolgung
   ${catkin_LIBRARIES}
)
target_link_libraries(pfaderstellung
   ${catkin_LIBRARIES}
)
target_link_libraries(pfadverfolgung2
   ${catkin_LIBRARIES}
)
target_link_libraries(pfaderstellung2
   ${catkin_LIBRARIES}
)
target_link_libraries(pfadverfolgung3
   ${catkin_LIBRARIES}
)
target_link_libraries(pfaderstellung3
   ${catkin_LIBRARIES}
)
