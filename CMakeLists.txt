set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)


cmake_minimum_required(VERSION 2.8.3)
project(optitrack_person)

set(ROS_DEPS
  roscpp
  tf
)

## set of packages whose message types this package use
set(MSG_DEP_SET
  geometry_msgs
  spencer_tracking_msgs
#  optitrack
)

set(MSG_FILES
  # these messages are copied from optitrack_genom, becasue optitrack_genom is built as a dry package
  or_pose_estimator_state.msg
  or_t3d_acc_cov.msg
  or_t3d_acc.msg
  or_t3d_pos_cov.msg
  or_t3d_pos.msg
  or_t3d_vel_cov.msg
  or_t3d_vel.msg
  or_time_ts.msg
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${ROS_DEPS}
  ${MSG_DEP_SET}
)

## add message, serviece and action files
add_message_files(DIRECTORY msg FILES ${MSG_FILES})

## generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MSG_DEP_SET}
)

## catkin specific configuration ##

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES optitrack_person
  CATKIN_DEPENDS
    message_runtime
    ${ROS_DEPS}
    ${MSG_DEP_SET}
#  DEPENDS system_lib
)



## Build ##

## include catking include directories
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## declare a cpp executable
add_executable(optitrack_person_node src/optitrack_person_node.cpp)

## cmake target dependencies of the executable/library
add_dependencies(optitrack_person_node optitrack_person_generate_messages_cpp)

## build config headers
add_dependencies(optitrack_person_node optitrack_person_gencfg)

## libraries to link a library or executable target against
target_link_libraries(optitrack_person_node
  ${catkin_LIBRARIES}
)



## Install ##

## executables and/or libraries for installation
install(TARGETS optitrack_person_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## other files for installation (launch and bag files)
install(PROGRAMS
  scripts/receive_optitrack.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/connect_optitrack.tcl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
