cmake_minimum_required(VERSION 2.8.3)
PROJECT (backend_S)


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
  interactive_markers
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
  src/entry_point.cu
  src/planner_interface.cu 
  src/planner.cu 
  src/simulator.cu 
  src/flock_biased.cu 
  src/formation_control.cu 
  src/rendezvous.cu 
  src/planner_pipeline.cu 
  src/simulator_pipeline.cu
  src/cub/cub.cuh
)

CUDA_ADD_LIBRARY(CudaLib2 ${LIB_TYPE} ${CUDA_SOURCE})
target_link_libraries(CudaLib2 cudadevrt cudart)

#add_message_files(
#  FILES
#)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs custom_messages visualization_msgs

)

include_directories(

  ${catkin_INCLUDE_DIRS}

)


ADD_EXECUTABLE(backend_S_node src/main.cpp)

ADD_DEPENDENCIES(backend_S_node CudaLib2)

TARGET_LINK_LIBRARIES(backend_S_node

   ${catkin_LIBRARIES}

   ${PCL_LIBRARIES}

   CudaLib2

)

