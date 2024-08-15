cmake_minimum_required(VERSION 3.15.0)
project(mrs_errorgraph_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-save-temps)
add_definitions(-O0)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  message_generation
  nodelet
  mrs_msgs
  mrs_lib
  )

set(LIBRARIES
  MRSErrorgraphViewer
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# messages
add_message_files(DIRECTORY msg FILES
  ErrorgraphNodeID.msg
  ErrorgraphError.msg
  ErrorgraphElement.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  mrs_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  )

## | ------------ MRSErrorgraphViewer ------------ |

## Declare a C++ library
add_library(MRSErrorgraphViewer
  src/errorgraph_viewer.cpp
  src/errorgraph.cpp
  )

add_dependencies(MRSErrorgraphViewer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(MRSErrorgraphViewer
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/mrs_errorgraph_viewer/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
