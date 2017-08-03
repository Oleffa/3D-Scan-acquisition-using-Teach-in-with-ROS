# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "riegl: 2 messages, 6 services")

set(MSG_I_FLAGS "-Iriegl:/home/oleffa/catkin_ws/src/riegl/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(riegl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv" ""
)

get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv" NAME_WE)
add_custom_target(_riegl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "riegl" "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)
_generate_msg_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)

### Generating Services
_generate_srv_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)
_generate_srv_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)
_generate_srv_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)
_generate_srv_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)
_generate_srv_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)
_generate_srv_cpp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
)

### Generating Module File
_generate_module_cpp(riegl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(riegl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(riegl_generate_messages riegl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv" NAME_WE)
add_dependencies(riegl_generate_messages_cpp _riegl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(riegl_gencpp)
add_dependencies(riegl_gencpp riegl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS riegl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)
_generate_msg_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)

### Generating Services
_generate_srv_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)
_generate_srv_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)
_generate_srv_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)
_generate_srv_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)
_generate_srv_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)
_generate_srv_eus(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
)

### Generating Module File
_generate_module_eus(riegl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(riegl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(riegl_generate_messages riegl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv" NAME_WE)
add_dependencies(riegl_generate_messages_eus _riegl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(riegl_geneus)
add_dependencies(riegl_geneus riegl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS riegl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)
_generate_msg_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)

### Generating Services
_generate_srv_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)
_generate_srv_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)
_generate_srv_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)
_generate_srv_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)
_generate_srv_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)
_generate_srv_lisp(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
)

### Generating Module File
_generate_module_lisp(riegl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(riegl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(riegl_generate_messages riegl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv" NAME_WE)
add_dependencies(riegl_generate_messages_lisp _riegl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(riegl_genlisp)
add_dependencies(riegl_genlisp riegl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS riegl_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)
_generate_msg_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)

### Generating Services
_generate_srv_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)
_generate_srv_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)
_generate_srv_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)
_generate_srv_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)
_generate_srv_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)
_generate_srv_py(riegl
  "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
)

### Generating Module File
_generate_module_py(riegl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(riegl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(riegl_generate_messages riegl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/selection.srv" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/scanparams.srv" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglTime.msg" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/Command.srv" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/msg/RieglStatus.msg" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/angle.srv" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/progress.srv" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/oleffa/catkin_ws/src/riegl/srv/inclination.srv" NAME_WE)
add_dependencies(riegl_generate_messages_py _riegl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(riegl_genpy)
add_dependencies(riegl_genpy riegl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS riegl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/riegl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(riegl_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/riegl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(riegl_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/riegl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(riegl_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/riegl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(riegl_generate_messages_py std_msgs_generate_messages_py)
