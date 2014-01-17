# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nvbg)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS uscauv_common roscpp sbl_msgs bml_cpp)
find_package(Boost REQUIRED)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS uscauv_common roscpp sbl_msgs bml_cpp
    INCLUDE_DIRS include 
    LIBRARIES 
)

# add_definitions(-lxerces-c)
add_executable( nvbg_server nodes/nvbg_server_node.cpp )
target_link_libraries(nvbg_server ${Boost_LIBRARIES} ${catkin_LIBRARIES} xerces-c)