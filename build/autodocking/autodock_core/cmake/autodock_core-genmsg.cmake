# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "autodock_core: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iautodock_core:/root/BumbleBot_WS/devel/share/autodock_core/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(autodock_core_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" "autodock_core/AutoDockingActionGoal:autodock_core/AutoDockingFeedback:autodock_core/AutoDockingActionResult:autodock_core/AutoDockingActionFeedback:autodock_core/AutoDockingGoal:std_msgs/Header:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:autodock_core/AutoDockingResult"
)

get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" "autodock_core/AutoDockingGoal:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" "std_msgs/Header:actionlib_msgs/GoalStatus:autodock_core/AutoDockingResult:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" "autodock_core/AutoDockingFeedback:std_msgs/Header:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" ""
)

get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" ""
)

get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" NAME_WE)
add_custom_target(_autodock_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autodock_core" "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)
_generate_msg_cpp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
)

### Generating Services

### Generating Module File
_generate_module_cpp(autodock_core
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(autodock_core_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(autodock_core_generate_messages autodock_core_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_cpp _autodock_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autodock_core_gencpp)
add_dependencies(autodock_core_gencpp autodock_core_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autodock_core_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)
_generate_msg_eus(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
)

### Generating Services

### Generating Module File
_generate_module_eus(autodock_core
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(autodock_core_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(autodock_core_generate_messages autodock_core_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_eus _autodock_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autodock_core_geneus)
add_dependencies(autodock_core_geneus autodock_core_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autodock_core_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)
_generate_msg_lisp(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
)

### Generating Services

### Generating Module File
_generate_module_lisp(autodock_core
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(autodock_core_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(autodock_core_generate_messages autodock_core_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_lisp _autodock_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autodock_core_genlisp)
add_dependencies(autodock_core_genlisp autodock_core_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autodock_core_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)
_generate_msg_nodejs(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
)

### Generating Services

### Generating Module File
_generate_module_nodejs(autodock_core
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(autodock_core_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(autodock_core_generate_messages autodock_core_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_nodejs _autodock_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autodock_core_gennodejs)
add_dependencies(autodock_core_gennodejs autodock_core_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autodock_core_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)
_generate_msg_py(autodock_core
  "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
)

### Generating Services

### Generating Module File
_generate_module_py(autodock_core
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(autodock_core_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(autodock_core_generate_messages autodock_core_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingAction.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingActionFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingGoal.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingResult.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/BumbleBot_WS/devel/share/autodock_core/msg/AutoDockingFeedback.msg" NAME_WE)
add_dependencies(autodock_core_generate_messages_py _autodock_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autodock_core_genpy)
add_dependencies(autodock_core_genpy autodock_core_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autodock_core_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autodock_core
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(autodock_core_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(autodock_core_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autodock_core
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(autodock_core_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(autodock_core_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autodock_core
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(autodock_core_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(autodock_core_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autodock_core
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(autodock_core_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(autodock_core_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autodock_core
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(autodock_core_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(autodock_core_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
