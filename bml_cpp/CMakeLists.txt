cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################################################################
# Call XSD to generate C++ BMl parser from XML Schema ##########
################################################################
# Not using an absolute path for the output causes weird behavior where the output is relative to
# WORKING_DIRECTORY but the later call to rosbuild_add_library assumes that it is in
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/bml-1.0.cpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/bml-1.0.h
  COMMAND xsd cxx-tree --hxx-suffix .h --cxx-suffix .cpp --include-with-brackets 
  --include-prefix "bml_cpp/" --generate-serialization --root-element bml
  --namespace-map "http://www.bml-initiative.org/bml/bml-1.0=bml" 
  ${PROJECT_SOURCE_DIR}/xsd/bml-1.0.xsd
  COMMAND mv bml-1.0.h ${PROJECT_SOURCE_DIR}/include/bml_cpp
  COMMAND mv bml-1.0.cpp ${PROJECT_SOURCE_DIR}/src
  DEPENDS xsd/bml-1.0.xsd
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Generating BML parser source code"
  VERBATIM)

rosbuild_add_library(${PROJECT_NAME} src/bml-1.0.cpp)
