# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "swarm_robot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iswarm_robot:/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(swarm_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" NAME_WE)
add_custom_target(_swarm_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_robot" "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(swarm_robot
  "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_robot
)

### Generating Services

### Generating Module File
_generate_module_cpp(swarm_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(swarm_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(swarm_robot_generate_messages swarm_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" NAME_WE)
add_dependencies(swarm_robot_generate_messages_cpp _swarm_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_robot_gencpp)
add_dependencies(swarm_robot_gencpp swarm_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_robot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(swarm_robot
  "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_robot
)

### Generating Services

### Generating Module File
_generate_module_eus(swarm_robot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_robot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(swarm_robot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(swarm_robot_generate_messages swarm_robot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" NAME_WE)
add_dependencies(swarm_robot_generate_messages_eus _swarm_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_robot_geneus)
add_dependencies(swarm_robot_geneus swarm_robot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_robot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(swarm_robot
  "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_robot
)

### Generating Services

### Generating Module File
_generate_module_lisp(swarm_robot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_robot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(swarm_robot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(swarm_robot_generate_messages swarm_robot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" NAME_WE)
add_dependencies(swarm_robot_generate_messages_lisp _swarm_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_robot_genlisp)
add_dependencies(swarm_robot_genlisp swarm_robot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_robot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(swarm_robot
  "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_robot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(swarm_robot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_robot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(swarm_robot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(swarm_robot_generate_messages swarm_robot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" NAME_WE)
add_dependencies(swarm_robot_generate_messages_nodejs _swarm_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_robot_gennodejs)
add_dependencies(swarm_robot_gennodejs swarm_robot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_robot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(swarm_robot
  "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_robot
)

### Generating Services

### Generating Module File
_generate_module_py(swarm_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(swarm_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(swarm_robot_generate_messages swarm_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wade/SJTU-swarm/swarm_ws/src/swarm_robot/msg/pos_info.msg" NAME_WE)
add_dependencies(swarm_robot_generate_messages_py _swarm_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_robot_genpy)
add_dependencies(swarm_robot_genpy swarm_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(swarm_robot_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(swarm_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_robot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(swarm_robot_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(swarm_robot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_robot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(swarm_robot_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(swarm_robot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_robot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(swarm_robot_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(swarm_robot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_robot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(swarm_robot_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(swarm_robot_generate_messages_py std_msgs_generate_messages_py)
endif()
