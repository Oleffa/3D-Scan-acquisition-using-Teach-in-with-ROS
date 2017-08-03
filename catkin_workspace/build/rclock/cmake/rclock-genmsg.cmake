# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rclock: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rclock_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv" NAME_WE)
add_custom_target(_rclock_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rclock" "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rclock
  "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rclock
)

### Generating Module File
_generate_module_cpp(rclock
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rclock
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rclock_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rclock_generate_messages rclock_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv" NAME_WE)
add_dependencies(rclock_generate_messages_cpp _rclock_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rclock_gencpp)
add_dependencies(rclock_gencpp rclock_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rclock_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(rclock
  "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rclock
)

### Generating Module File
_generate_module_eus(rclock
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rclock
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rclock_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rclock_generate_messages rclock_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv" NAME_WE)
add_dependencies(rclock_generate_messages_eus _rclock_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rclock_geneus)
add_dependencies(rclock_geneus rclock_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rclock_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rclock
  "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rclock
)

### Generating Module File
_generate_module_lisp(rclock
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rclock
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rclock_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rclock_generate_messages rclock_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv" NAME_WE)
add_dependencies(rclock_generate_messages_lisp _rclock_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rclock_genlisp)
add_dependencies(rclock_genlisp rclock_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rclock_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rclock
  "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rclock
)

### Generating Module File
_generate_module_py(rclock
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rclock
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rclock_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rclock_generate_messages rclock_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/rclock/srv/logDir.srv" NAME_WE)
add_dependencies(rclock_generate_messages_py _rclock_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rclock_genpy)
add_dependencies(rclock_genpy rclock_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rclock_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rclock)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rclock
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(rclock_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rclock)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rclock
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(rclock_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rclock)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rclock
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(rclock_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rclock)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rclock\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rclock
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(rclock_generate_messages_py std_msgs_generate_messages_py)
