# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


cmake_minimum_required(VERSION 2.8.3)
project(optitrack_human)

set(ROS_DEPS
  roscpp
)

## set of packages whose message types this package use
set(MSG_DEP_SET
  spencer_tracking_msgs
)

#set(MSG_FILES
#)

#set(SRV_FILES
#)

#set(ACT_FILES
#)

find_package(catkin REQUIRED COMPONENTS
#  message_generation
  ${ROS_DEPS}
  ${MSG_DEP_SET}
)
# find_package(Boost REQUIRED COMPONENTS system)

## add message, serviece and action files
#add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#add_service_files(DIRECTORY srv FILES ${SRV_FILES})
#add_action_files(DIRECTORY action FILES ${ACT_FILES})

## generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  ${MSG_DEP_SET}
#)

## catkin specific configuration ##

catkin_package(
  INCLUDE_DIRS 
    include
#  LIBRARIES optitrack_human
  CATKIN_DEPENDS
    #message_runtime
    ${ROS_DEPS} 
    ${MSG_DEP_SET} 
    optitrack
#  DEPENDS system_lib
)



## Build ##

## additional locations of header files
## because cakin cannot depend on rosbuild!!
include_directories(/home/hkhambha/openrobots/devel/include)

## include catking include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## declare a cpp library
# add_library(optitrack_human
#   src/${PROJECT_NAME}/optitrack_human.cpp
# )

## declare a cpp executable
add_executable(optitrack_human_node src/optitrack_human_node.cpp)

## cmake target dependencies of the executable/library
# add_dependencies(optitrack_human_node optitrack_human_generate_messages_cpp)

## libraries to link a library or executable target against
target_link_libraries(optitrack_human_node
  ${catkin_LIBRARIES}
)



## Install ##

## executable scripts (python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## executables and/or libraries for installation
# install(TARGETS optitrack_human optitrack_human_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## cpp-header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## other files for installation (launch and bag files)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



## Testing ##

## add catch based cpp test target
# catkin_add_gtest(${PROJECT_NAME}-test test/test_optitrack_human.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
