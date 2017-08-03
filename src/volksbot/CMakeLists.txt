cmake_minimum_required(VERSION 2.8.3)
project(volksbot)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
  tf
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
  pose2d.msg
  ticks.msg
  vels.msg
)

add_service_files(
  FILES
  velocities.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime roscpp 
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories(
  launch
)

add_library(VMCLIB
  src/Adapter/CComAdapter.cpp
  src/LayerClasses/CVmc.cpp
  src/SupportClasses/CData.cpp
  src/SupportClasses/CRequest.cpp
  src/LayerClasses/CCommunicationLayer.cpp
  src/StorageClasses/CMotor.cpp
  src/SupportClasses/CError.cpp
  src/SupportClasses/CSendTwo.cpp
  src/LayerClasses/CTranslationLayer.cpp
  src/StorageClasses/CStorage.cpp
  src/SupportClasses/CMessage.cpp
  src/SupportClasses/CTimer.cpp
  src/LayerClasses/CvmcAPI.cpp
  src/SupportClasses/CChannel.cpp
  src/SupportClasses/CMultisend.cpp
  src/SupportClasses/CTimestamp.cpp
)
add_dependencies(VMCLIB volksbot_generate_messages_cpp volksbot_generate_services_cpp) 


add_library(odo
  src/odometry.cc
)
add_dependencies(odo volksbot_generate_messages_cpp volksbot_generate_services_cpp) 

add_executable(volksbot src/volksbot.cc)
set_target_properties(volksbot PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(volksbot
  ${catkin_LIBRARIES}
  VMCLIB
)
add_dependencies(volksbot volksbot_generate_messages_cpp volksbot_generate_services_cpp) 

add_executable(kbcontrol src/kbcontrol.cc)
set_target_properties(kbcontrol PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(kbcontrol
  ${catkin_LIBRARIES}
)
add_dependencies(kbcontrol volksbot_generate_messages_cpp volksbot_generate_services_cpp)

add_executable(odometry 
  src/odometry.cc 
  src/odometry_main.cc
)
set_target_properties(odometry PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(odometry
  ${catkin_LIBRARIES}
)
add_dependencies(odometry volksbot_generate_messages_cpp volksbot_generate_services_cpp)

add_executable(calibration src/calibration.cc)
set_target_properties(calibration PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(calibration
  ${catkin_LIBRARIES}
)
add_dependencies(calibration volksbot_generate_messages_cpp volksbot_generate_services_cpp)

add_executable(jscontrol
  src/joystick/jscontrol.cc
  src/joystick/joystick.cc
  src/joystick/firejet.cc
  src/joystick/predator.cc
  src/joystick/logitechf710.cc
)
set_target_properties(jscontrol PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(jscontrol
  ${catkin_LIBRARIES}
)
add_dependencies(jscontrol volksbot_generate_messages_cpp volksbot_generate_services_cpp)


MESSAGE(STATUS "STATUS: ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
MESSAGE(STATUS "STATUS: ${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
MESSAGE(STATUS "STATUS: include/${PROJECT_NAME}")
MESSAGE(STATUS "STATUS: ${CATKIN_DEVEL_PREFIX}/include/")

#install(DIRECTORY include/${PROJECT_NAME}
#	DESTINATION ${CATKIN_DEVEL_PREFIX}/include/
#	PATTERN ".svn" EXCLUDE)

#DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/volksbot
#)

install(TARGETS odo VMCLIB DESTINATION lib EXPORT volksbot-targets)
install(EXPORT volksbot-targets DESTINATION lib/volksbot)
