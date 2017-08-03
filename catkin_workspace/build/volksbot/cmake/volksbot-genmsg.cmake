# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "volksbot: 3 messages, 1 services")

set(MSG_I_FLAGS "-Ivolksbot:/home/oleffa/catkin_ws/src/volksbot/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(volksbot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv" NAME_WE)
add_custom_target(_volksbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "volksbot" "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg" NAME_WE)
add_custom_target(_volksbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "volksbot" "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg" NAME_WE)
add_custom_target(_volksbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "volksbot" "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg" NAME_WE)
add_custom_target(_volksbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "volksbot" "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot
)
_generate_msg_cpp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot
)
_generate_msg_cpp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot
)

### Generating Services
_generate_srv_cpp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot
)

### Generating Module File
_generate_module_cpp(volksbot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(volksbot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(volksbot_generate_messages volksbot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv" NAME_WE)
add_dependencies(volksbot_generate_messages_cpp _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_cpp _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_cpp _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_cpp _volksbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(volksbot_gencpp)
add_dependencies(volksbot_gencpp volksbot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS volksbot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot
)
_generate_msg_eus(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot
)
_generate_msg_eus(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot
)

### Generating Services
_generate_srv_eus(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot
)

### Generating Module File
_generate_module_eus(volksbot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(volksbot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(volksbot_generate_messages volksbot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv" NAME_WE)
add_dependencies(volksbot_generate_messages_eus _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_eus _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_eus _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_eus _volksbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(volksbot_geneus)
add_dependencies(volksbot_geneus volksbot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS volksbot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot
)
_generate_msg_lisp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot
)
_generate_msg_lisp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot
)

### Generating Services
_generate_srv_lisp(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot
)

### Generating Module File
_generate_module_lisp(volksbot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(volksbot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(volksbot_generate_messages volksbot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv" NAME_WE)
add_dependencies(volksbot_generate_messages_lisp _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_lisp _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_lisp _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_lisp _volksbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(volksbot_genlisp)
add_dependencies(volksbot_genlisp volksbot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS volksbot_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot
)
_generate_msg_py(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot
)
_generate_msg_py(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot
)

### Generating Services
_generate_srv_py(volksbot
  "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot
)

### Generating Module File
_generate_module_py(volksbot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(volksbot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(volksbot_generate_messages volksbot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/srv/velocities.srv" NAME_WE)
add_dependencies(volksbot_generate_messages_py _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/ticks.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_py _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/pose2d.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_py _volksbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/volksbot/msg/vels.msg" NAME_WE)
add_dependencies(volksbot_generate_messages_py _volksbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(volksbot_genpy)
add_dependencies(volksbot_genpy volksbot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS volksbot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/volksbot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(volksbot_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/volksbot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(volksbot_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/volksbot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(volksbot_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/volksbot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(volksbot_generate_messages_py std_msgs_generate_messages_py)
