cmake_minimum_required(VERSION 2.8.3)
project(speech_realizer)

#############################################################
# Note: Do not build with C++11 flags under any circumstances!
# This breaks some festival functionality ###################
#############################################################

#############################################################
# Another note: You need to find /usr/include/<estools dir>/ling_class/EST_Relation_list.h
# and change the '#if 0' in it to a '#if 1' so all the functions we need are defined
# <estools dir> is either 'estools' or 'speech_tools'
#############################################################

# set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_VERBOSE_MAKEFILE ON)
# set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL REQUIRED sdl)

set(festival_INCLUDE_DIRS /usr/include/estools /usr/include/festival /usr/include/speech_tools)
set(festival_LIBRARIES Festival estools estbase eststring esd ncurses)

add_message_files(FILES
  TimedWord.msg
)

add_service_files(FILES
  GetWordTimings.srv
)

add_action_files(DIRECTORY action FILES SayText.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${festival_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable( speech_realizer nodes/speech_realizer_node.cpp )
add_dependencies(speech_realizer ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(speech_realizer ${catkin_LIBRARIES} ${festival_LIBRARIES} ${Boost_LIBRARIES} ${SDL_LIBRARIES} SDL_mixer)
