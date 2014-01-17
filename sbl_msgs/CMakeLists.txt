cmake_minimum_required(VERSION 2.8.3)
project(sbl_msgs)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs)

#uncomment if you have defined messages
add_message_files(
  FILES
  SimpleNVBGRequest.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime roscpp std_msgs
    INCLUDE_DIRS
    LIBRARIES
)