cmake_minimum_required(VERSION 2.8.3)
project(bag2scans)

set(ADDITIONAL_FLAGS " -O3 -std=c++0x -msse3 -fPIC -Wall -finline-functions -Wno-unused-but-set-variable -Wno-write-strings -Wno-char-subscripts -Wno-unused-result")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(THREEDTK_PATH "/home/oleffa/3DTK/slam6d-code/")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  rosbag2
  volksbot 
  riegl REQUIRED
  tf
  laser_geometry
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig)

pkg_check_modules(BFL REQUIRED bfl)

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=4 -DOPENMP_NUM_THREADS=4 ${OpenMP_CXX_FLAGS} -DOPENMP ")

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime rosbag eigen bfl riegl volksbot
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${riegl_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
  ${volksbot_INCLUDE_DIRS}
  ${THREEDTK_PATH}/include
  ${THREEDTK_PATH}/3rdparty
)

link_directories(
  ${THREEDTK_PATH}/lib
  ${THREEDTK_PATH}/3rdparty/riegl
  ${BFL_LIBRARY_DIRS}
  ${volksbot_LIBRARY_DIRS}
  /home/oleffa/catkin_ws/devel/lib
)

add_executable(bag2scan src/bag2scan.cc src/importer.cc src/calibration.cc src/nrutil.cc src/powell.cc src/ekf_filter.cc src/my_robot_pose_ekf/nonlinearanalyticconditionalgaussianodo.cpp src/my_robot_pose_ekf/odom_estimation.cpp)
set_target_properties(bag2scan PROPERTIES LINK_FLAGS "-pthread" COMPILE_FLAGS "-Wno-extra")
target_link_libraries(bag2scan
  ${catkin_LIBRARIES}
  scan_s
  ${riegl_LIBRARIES}
  scanlib-mt
  boost_system-mt-1_43_0-vns
  ${BFL_LIBRARIES}
  newmat
  odo
  ${laser_geometry_LIBRARIES}
  ${volksbot_LIBRARIES}
)

#add_executable(correction src/correction.cc src/linescan.cc src/continuousreg.cc src/lum6Deuler.cc src/ghelix6DQ2.cc src/simplereg.cc src/gapx6D.cc)
#set_target_properties(correction PROPERTIES LINK_FLAGS "-pthread" COMPILE_FLAGS "-Wno-extra")
#target_link_libraries(correction
#  ${catkin_LIBRARIES}
#  scan_s
#  newmat_s
#)

add_executable(record src/record.cc)
set_target_properties(record PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(record
  ${catkin_LIBRARIES}
)

#add_executable(join src/join.cc src/linescan.cc)
#set_target_properties(join PROPERTIES LINK_FLAGS "-pthread" COMPILE_FLAGS "-Wno-extra")
#target_link_libraries(join
#  ${catkin_LIBRARIES}
#  scan_s
#)

#add_executable(convert src/convert.cc)
#set_target_properties(convert PROPERTIES LINK_FLAGS "-pthread")
#target_link_libraries(convert
#  ${catkin_LIBRARIES}
#  scan_s
#)

#add_executable(convertriegl src/convertriegl.cc)
#set_target_properties(convertriegl PROPERTIES LINK_FLAGS "-lrt -pthread")
#target_link_libraries(convertriegl
#  ${catkin_LIBRARIES}
#  scan_s
#  scanlib-mt
#  boost_system-mt-1_43_0-vns
#  ${riegl_LIBRARIES}
#)
