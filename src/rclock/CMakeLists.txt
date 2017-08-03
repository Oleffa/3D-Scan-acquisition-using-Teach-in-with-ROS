cmake_minimum_required(VERSION 2.8.3)
project(rclock)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
)

add_service_files(
  FILES
  logDir.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(timepublisher src/timepublisher.cc)
target_link_libraries(timepublisher ${catkin_LIBRARIES})
add_dependencies(timepublisher rclock_generate_messages_cpp rclock_generate_services_cpp)

add_executable(timesetter src/timesetter.cc)
target_link_libraries(timesetter ${catkin_LIBRARIES})
add_dependencies(timesetter rclock_generate_messages_cpp rclock_generate_services_cpp)
