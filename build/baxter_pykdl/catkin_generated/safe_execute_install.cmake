execute_process(COMMAND "/home/cc/ee106a/fa23/class/ee106a-aiq/Desktop/EECS-106A-Final-Project/build/baxter_pykdl/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106a/fa23/class/ee106a-aiq/Desktop/EECS-106A-Final-Project/build/baxter_pykdl/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
