cmake_minimum_required(VERSION 2.8.3)
project(ros_openpose)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  message_generation
)

## Make sure 'FindGFlags.cmake' and 'FindGlog.cmake' are visible to cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

## Add OpenPose and other dependencies
find_package(GFlags)
find_package(Glog)
find_package(OpenCV REQUIRED)
find_package(OpenPose REQUIRED)
find_package(OpenMP)
find_package(Threads REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Pass in the version of OpenPose
add_definitions( -DOpenPose_VERSION_MAJOR=${OpenPose_VERSION_MAJOR} )
add_definitions( -DOpenPose_VERSION_MINOR=${OpenPose_VERSION_MINOR} )
add_definitions( -DOpenPose_VERSION_PATCH=${OpenPose_VERSION_PATCH} )

## Print OpenPose version in the terminal
MESSAGE(STATUS "OpenPose VERSION: " ${OpenPose_VERSION})

add_message_files(FILES Pixel.msg BodyPart.msg Skeleton.msg SkeletonArray.msg)

generate_messages(DEPENDENCIES geometry_msgs)

# catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Build
# Specify additional locations of header files
include_directories(
  include
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

## Path to Caffe lib i.e., libcaffe.so
set(CAFFE_LIB_FOLDER ${OpenPose_DIR}/../../lib)
link_directories(${CAFFE_LIB_FOLDER})

catkin_install_python(PROGRAMS 
                      scripts/multi_openpose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

# add_executable(${PROJECT_NAME}_node_sync src/open_vico_openpose_node.cpp)
# set_target_properties(${PROJECT_NAME}_node_sync PROPERTIES OUTPUT_NAME "open_vico_openpose")
# add_dependencies(${PROJECT_NAME}_node_sync ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node_sync ${OpenPose_LIBS} gflags ${GFLAGS_LIBRARY} ${GLOG_LIBRARY} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/rosOpenposeAsync.cpp src/cameraReader.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME "ros_openpose")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${OpenPose_LIBS} gflags ${GFLAGS_LIBRARY} ${GLOG_LIBRARY} ${OpenCV_LIBS} ${catkin_LIBRARIES})