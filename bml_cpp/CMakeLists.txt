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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries

set_directory_properties(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES bml-1.0.cpp
  ADDITIONAL_MAKE_CLEAN_FILES bml-1.0.h )

add_custom_target(BML_SOURCE ALL
  xsd cxx-tree --hxx-suffix .h --cxx-suffix .cpp --include-with-brackets 
  --include-prefix "bml_cpp/" --generate-serialization --root-element bml
  --namespace-map "http://www.bml-initiative.org/bml/bml-1.0=bml" 
  ${PROJECT_SOURCE_DIR}/xsd/bml-1.0.xsd && 
  mv bml-1.0.h ${PROJECT_SOURCE_DIR}/include/bml_cpp && 
  mv bml-1.0.cpp ${PROJECT_SOURCE_DIR}/src
  DEPENDS xsd/bml-1.0.xsd
  COMMENT "Generating BML parser source code"
  VERBATIM)
add_custom_target(BML_SOURCE_TARGET DEPENDS BML_SOURCE)

# Ensure that BML_SOURCE is generated before we try to build anything
add_dependencies(rosbuild_precompile BML_SOURCE)

rosbuild_add_library(${PROJECT_NAME} src/bml-1.0.cpp)
