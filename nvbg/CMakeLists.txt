# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(nvbg)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS uscauv_common roscpp sbl_msgs bml_cpp std_msgs)
find_package(Boost REQUIRED)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(xerces_LIBRARIES xerces-c)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS uscauv_common roscpp sbl_msgs bml_cpp std_msgs
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
)

add_definitions(-DUSCAUV_PARAM_LOADER_DISALLOW_EMPTY_MAPS)
add_definitions(-DUSCAUV_PARAM_LOADER_DISALLOW_EMPTY_VECTORS)

add_library(${PROJECT_NAME} src/types.cpp src/param_conversions.cpp src/bml_generation.cpp 
  src/parsing.cpp src/constrained.cpp src/resolve_constraints.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${xerces_LIBRARIES})

add_executable(nvbg_server nodes/nvbg_server_node.cpp )
target_link_libraries(nvbg_server ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${xerces_LIBRARIES})
