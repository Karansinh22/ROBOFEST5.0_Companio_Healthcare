execute_process(COMMAND "/root/BumbleBot_WS/build/robot_upstart/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/root/BumbleBot_WS/build/robot_upstart/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
