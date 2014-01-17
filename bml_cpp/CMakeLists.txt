cmake_minimum_required(VERSION 2.8.3)
project(bml_cpp)
find_package(catkin REQUIRED COMPONENTS)

################################################################
# Call XSD to generate C++ BMl parser from XML Schema ##########
################################################################
# Not using an absolute path for the output causes weird behavior where the output is relative to
# WORKING_DIRECTORY but the later call to rosbuild_add_library assumes that it is in
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/bml-1.0.cpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/bml-1.0.h
  COMMAND xsdcxx cxx-tree --hxx-suffix .h --cxx-suffix .cpp --include-with-brackets 
  --include-prefix "bml_cpp/" --generate-serialization --root-element bml
  --namespace-map "http://www.bml-initiative.org/bml/bml-1.0=bml" 
  ${PROJECT_SOURCE_DIR}/xsd/bml-1.0.xsd
  COMMAND mv bml-1.0.h ${PROJECT_SOURCE_DIR}/include/bml_cpp
  COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/src
  COMMAND mv bml-1.0.cpp ${PROJECT_SOURCE_DIR}/src
  DEPENDS xsd/bml-1.0.xsd
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Generating BML parser source code"
  VERBATIM)

add_definitions(-lxerces-c)
include_directories(include)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/bml-1.0.cpp)