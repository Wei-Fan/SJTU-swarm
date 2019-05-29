# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "viconros: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iviconros:/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(viconros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" NAME_WE)
add_custom_target(_viconros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "viconros" "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" "std_msgs/Header:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(viconros
  "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/viconros
)

### Generating Services

### Generating Module File
_generate_module_cpp(viconros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/viconros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(viconros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(viconros_generate_messages viconros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" NAME_WE)
add_dependencies(viconros_generate_messages_cpp _viconros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(viconros_gencpp)
add_dependencies(viconros_gencpp viconros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS viconros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(viconros
  "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/viconros
)

### Generating Services

### Generating Module File
_generate_module_eus(viconros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/viconros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(viconros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(viconros_generate_messages viconros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" NAME_WE)
add_dependencies(viconros_generate_messages_eus _viconros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(viconros_geneus)
add_dependencies(viconros_geneus viconros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS viconros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(viconros
  "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/viconros
)

### Generating Services

### Generating Module File
_generate_module_lisp(viconros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/viconros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(viconros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(viconros_generate_messages viconros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" NAME_WE)
add_dependencies(viconros_generate_messages_lisp _viconros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(viconros_genlisp)
add_dependencies(viconros_genlisp viconros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS viconros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(viconros
  "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/viconros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(viconros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/viconros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(viconros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(viconros_generate_messages viconros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" NAME_WE)
add_dependencies(viconros_generate_messages_nodejs _viconros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(viconros_gennodejs)
add_dependencies(viconros_gennodejs viconros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS viconros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(viconros
  "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/viconros
)

### Generating Services

### Generating Module File
_generate_module_py(viconros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/viconros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(viconros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(viconros_generate_messages viconros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/viconros/msg/viconmocap.msg" NAME_WE)
add_dependencies(viconros_generate_messages_py _viconros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(viconros_genpy)
add_dependencies(viconros_genpy viconros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS viconros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/viconros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/viconros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(viconros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(viconros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/viconros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/viconros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(viconros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(viconros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/viconros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/viconros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(viconros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(viconros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/viconros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/viconros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(viconros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(viconros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/viconros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/viconros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/viconros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(viconros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(viconros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
