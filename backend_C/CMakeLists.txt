cmake_minimum_required(VERSION 2.8.3)

PROJECT (backend_C)

FIND_PACKAGE(CUDA REQUIRED)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS

  roscpp

  rospy

  std_msgs
  custom_messages
  visualization_msgs
)



SET(CUDA_NVCC_FLAGS "-std=c++11 -arch=sm_35" CACHE STRING "nvcc flags" FORCE)

SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

SET(LIB_TYPE STATIC) 

CUDA_ADD_LIBRARY(CudaLib ${LIB_TYPE} src/smha_final.cu)


#add_message_files(
#  FILES
#)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs custom_messages visualization_msgs

)

include_directories(

  ${catkin_INCLUDE_DIRS}

)


ADD_EXECUTABLE(backend_C_node src/main.cpp)

ADD_DEPENDENCIES(backend_C_node CudaLib)

TARGET_LINK_LIBRARIES(backend_C_node

   ${catkin_LIBRARIES}

   ${PCL_LIBRARIES}

   CudaLib

)

add_executable(backend_C_test src/test.cpp)
target_link_libraries(backend_C_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})



