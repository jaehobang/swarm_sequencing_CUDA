cmake_minimum_required(VERSION 2.8.3)
PROJECT (planner_compile)


FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_SEPARABLE_COMPILATION ON)
#include_directories(/usr/local/cuda/include)
#add_definitions(-DUSE_GPU)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  custom_messages
  visualization_msgs
)
add_compile_options(-std=c++11)
#add_compile_options(-ltbb)
#add_compile_options(-lrt)
#add_compile_options(-lcudart)
#add_compile_options(-lcudadevrt)

SET(CUDA_NVCC_FLAGS "-std=c++11;--relocatable-device-code=true;-arch=compute_61;-code=sm_61;-ltbb;-lrt;-lcudart;-lcudadevrt" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)

link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)


SET(LIB_TYPE STATIC)

set(CUDA_SOURCE
  src/smha_final.cu
)

CUDA_ADD_LIBRARY(CudaLib4 ${LIB_TYPE} ${CUDA_SOURCE})
target_link_libraries(CudaLib4 cudadevrt cudart)

#add_message_files(
#  FILES
#)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs custom_messages visualization_msgs

)

include_directories(

  ${catkin_INCLUDE_DIRS}

)


ADD_EXECUTABLE(planner_compile_node src/main.cpp)

ADD_DEPENDENCIES(planner_compile_node CudaLib4)

TARGET_LINK_LIBRARIES(planner_compile_node

   ${catkin_LIBRARIES}

   ${PCL_LIBRARIES}

   CudaLib4

)

