# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bachelorarbeit: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bachelorarbeit_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_bachelorarbeit_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bachelorarbeit" "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv" NAME_WE)
add_custom_target(_bachelorarbeit_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bachelorarbeit" "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bachelorarbeit
)
_generate_srv_cpp(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bachelorarbeit
)

### Generating Module File
_generate_module_cpp(bachelorarbeit
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bachelorarbeit
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bachelorarbeit_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bachelorarbeit_generate_messages bachelorarbeit_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_cpp _bachelorarbeit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_cpp _bachelorarbeit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bachelorarbeit_gencpp)
add_dependencies(bachelorarbeit_gencpp bachelorarbeit_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bachelorarbeit_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bachelorarbeit
)
_generate_srv_eus(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bachelorarbeit
)

### Generating Module File
_generate_module_eus(bachelorarbeit
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bachelorarbeit
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bachelorarbeit_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bachelorarbeit_generate_messages bachelorarbeit_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_eus _bachelorarbeit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_eus _bachelorarbeit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bachelorarbeit_geneus)
add_dependencies(bachelorarbeit_geneus bachelorarbeit_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bachelorarbeit_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bachelorarbeit
)
_generate_srv_lisp(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bachelorarbeit
)

### Generating Module File
_generate_module_lisp(bachelorarbeit
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bachelorarbeit
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bachelorarbeit_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bachelorarbeit_generate_messages bachelorarbeit_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_lisp _bachelorarbeit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_lisp _bachelorarbeit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bachelorarbeit_genlisp)
add_dependencies(bachelorarbeit_genlisp bachelorarbeit_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bachelorarbeit_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bachelorarbeit
)
_generate_srv_py(bachelorarbeit
  "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bachelorarbeit
)

### Generating Module File
_generate_module_py(bachelorarbeit
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bachelorarbeit
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bachelorarbeit_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bachelorarbeit_generate_messages bachelorarbeit_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_py _bachelorarbeit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/bachelorarbeit/srv/pressed.srv" NAME_WE)
add_dependencies(bachelorarbeit_generate_messages_py _bachelorarbeit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bachelorarbeit_genpy)
add_dependencies(bachelorarbeit_genpy bachelorarbeit_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bachelorarbeit_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bachelorarbeit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bachelorarbeit
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(bachelorarbeit_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bachelorarbeit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bachelorarbeit
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(bachelorarbeit_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bachelorarbeit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bachelorarbeit
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(bachelorarbeit_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bachelorarbeit)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bachelorarbeit\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bachelorarbeit
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(bachelorarbeit_generate_messages_py std_msgs_generate_messages_py)
