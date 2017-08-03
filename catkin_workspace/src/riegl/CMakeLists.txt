cmake_minimum_required(VERSION 2.8.3)
project(riegl)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
)

add_message_files(
  FILES
  RieglStatus.msg
  RieglTime.msg
)

add_service_files(
  FILES
  angle.srv
  scanparams.srv
  Command.srv
  selection.srv
  progress.srv
  inclination.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime roscpp
  INCLUDE_DIRS include
#  LIBRARIES scanlib-mt ctrllib-mt   boost_system-mt-1_43_0-vns
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories(
  lib
)

add_executable(rxplogger src/rxplogger.cc)
set_target_properties(rxplogger PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(rxplogger
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(rxplogger riegl_generate_messages_cpp riegl_generate_services_cpp) 

add_executable(raw2scan src/raw2scan.cc)
set_target_properties(raw2scan PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(raw2scan
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(rxplogger riegl_generate_messages_cpp riegl_generate_services_cpp)

add_executable(ctr src/rieglctrl.cc)
set_target_properties(ctr PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(ctr
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(ctr riegl_generate_messages_cpp riegl_generate_services_cpp)

add_executable(receiver src/receiver.cc)
set_target_properties(receiver PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(receiver riegl_generate_messages_cpp riegl_generate_services_cpp)

add_executable(receiver2D src/receiver2D.cc)
set_target_properties(receiver2D PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver2D
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(receiver2D riegl_generate_messages_cpp riegl_generate_services_cpp)

add_executable(receiver3D src/receiver3D.cc)
set_target_properties(receiver3D PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver3D
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(receiver3D riegl_generate_messages_cpp riegl_generate_services_cpp)

add_executable(receiver4D src/receiver4D.cc)
set_target_properties(receiver4D PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver4D
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(receiver4D riegl_generate_messages_cpp riegl_generate_services_cpp)

add_executable(tslogger src/tslogger.cc)
set_target_properties(tslogger PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(tslogger
  ${catkin_LIBRARIES}
  scanlib-mt
  ctrllib-mt
    boost_system-mt-1_43_0-vns
)
add_dependencies(tslogger riegl_generate_messages_cpp riegl_generate_services_cpp)

install(DIRECTORY include/${PROJECT_NAME}
	DESTINATION ${CATKIN_DEVEL_PREFIX}/include
	PATTERN ".svn" EXCLUDE)

install(DIRECTORY lib/
	#DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	DESTINATION ${CATKIN_DEVEL_PREFIX}/lib
	PATTERN ".svn" EXCLUDE)
