# check c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



cmake_minimum_required(VERSION 2.8.3)
project(optitrack_person)

set(ROS_DEPS
  roscpp
  dynamic_reconfigure
)

## set of packages whose message types this package use
set(MSG_DEP_SET
  geometry_msgs
  spencer_tracking_msgs
#  optitrack
)

set(MSG_FILES
  # these messages are copied from optitrack_genom, becasue optitrack_genom is built as a dry package
  or_pose_estimator_motion.msg
  or_pose_estimator_reference.msg
  or_pose_estimator_state.msg
  or_t3d_acc_cov.msg
  or_t3d_acc.msg
  or_t3d_pos_cov.msg
  or_t3d_pos.msg
  or_t3d_vel_cov.msg
  or_t3d_vel.msg
  or_time_ts.msg
)

#set(SRV_FILES
#)

#set(ACT_FILES
#)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${ROS_DEPS}
  ${MSG_DEP_SET}
)
# find_package(Boost REQUIRED COMPONENTS system)

## add message, serviece and action files
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#add_service_files(DIRECTORY srv FILES ${SRV_FILES})
#add_action_files(DIRECTORY action FILES ${ACT_FILES})

## generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MSG_DEP_SET}
)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/OptitrackPerson.cfg
)

## catkin specific configuration ##

catkin_package(
#  INCLUDE_DIRS
#    include
#  LIBRARIES optitrack_person
  CATKIN_DEPENDS
    message_runtime
    ${ROS_DEPS}
    ${MSG_DEP_SET}
#  DEPENDS system_lib
)



## Build ##

## additional locations of header files
# specific path because cakin cannot depend on rosbuild!!
#include_directories(${HOME}/openrobots/install/include)

## include catking include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## declare a cpp library
# add_library(optitrack_person
#   src/${PROJECT_NAME}/optitrack_person.cpp
# )

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

## executable scripts (python etc.) for installation
#install(PROGRAMS
#  scripts/some.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## executables and/or libraries for installation
install(TARGETS optitrack_person_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## cpp-header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

## other files for installation (launch and bag files)
install(FILES
  scripts/connect_optitrack.tcl
  scripts/receive_optitrack.sh
  launch/optitrack.launch
  launch/optitrack_person.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



## Testing ##

## add catch based cpp test target
# catkin_add_gtest(${PROJECT_NAME}-test test/test_optitrack_person.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
