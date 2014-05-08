cmake_minimum_required(VERSION 2.8.3)
project(gesture_creation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS interactive_markers roscpp visualization_msgs dynamic_reconfigure rospy std_msgs tf)

include_directories(include ${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs dynamic_reconfigure rospy std_msgs tf
#  DEPENDS system_lib
)

add_library(${PROJECT_NAME} src/bezier_gen.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})