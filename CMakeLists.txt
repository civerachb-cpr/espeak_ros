cmake_minimum_required(VERSION 2.8.3)
project(espeak_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(REQUIRED COMPONENTS
  std_msgs
)

find_path(LIBESPEAK_INCLUDE_DIRS
          NAMES speak_lib.h
          HINTS /usr/include/espeak)

find_library(LIBESPEAK_LIBRARIES
             NAMES espeak
             HINTS /usr/lib/ /usr/x86_64-linux-gnu/
             PATH_SUFFIXES lib)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS libespeak-dev
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBESPEAK_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

add_executable(espeak_node src/espeak_node.cpp)

target_link_libraries(espeak_node
  ${catkin_LIBRARIES}
  ${LIBESPEAK_LIBRARIES}
  pthread
)

install(TARGETS espeak_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
