cmake_minimum_required(VERSION 2.8.3)
project(bml_realizer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  bml_cpp
  nvbg
  roscpp
  uscauv_common
)



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
    CATKIN_DEPENDS bml_cpp nvbg roscpp uscauv_common
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DTF_EULER_DEFAULT_ZYX)
add_library(${PROJECT_NAME}
  src/realizable.cpp src/sequence_gesture.cpp src/bezier_gesture.cpp
  )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bml_realizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)