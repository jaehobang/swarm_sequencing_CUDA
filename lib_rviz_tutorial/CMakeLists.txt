## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(lib_rviz_tutorial)
add_compile_options(-std=c++11)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS rviz roscpp )
catkin_package(
  CATKIN_DEPENDS roscpp rviz
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDES})


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
add_definitions(${Qt5Widgets_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")


## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/myviz.cpp
  src/main.cpp
  src/myviz.h
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SOURCE_FILES}``.
add_executable(myviz ${SOURCE_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
target_link_libraries(myviz ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

add_executable(test111 src/test.cpp)
target_link_libraries(test111 ${catkin_LIBRARIES})


## Install
