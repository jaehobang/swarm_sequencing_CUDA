cmake_minimum_required(VERSION 2.8.3)

PROJECT (beginner_tutorials)

FIND_PACKAGE(CUDA REQUIRED)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS

  roscpp

  rospy

  std_msgs
  message_generation
)



SET(CUDA_NVCC_FLAGS "-std=c++11 -arch=sm_20" CACHE STRING "nvcc flags" FORCE)

SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

SET(LIB_TYPE STATIC) 

CUDA_ADD_LIBRARY(TestLib ${LIB_TYPE} src/helloWorld.cu)


add_message_files(
  FILES
  Num.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime

)

include_directories(

  ${catkin_INCLUDE_DIRS}

)


ADD_EXECUTABLE(beginner_tutorials_node src/main.cpp)

ADD_DEPENDENCIES(beginner_tutorials_node TestLib)

TARGET_LINK_LIBRARIES(beginner_tutorials_node

   ${catkin_LIBRARIES}

   ${PCL_LIBRARIES}

   TestLib

)
