cmake_minimum_required(VERSION 2.8.3)
project(image_out)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
  cv_bridge
  image_transport
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  msg2file.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(DEPENDS OpenCV)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(image_out src/image_out.cpp)
target_link_libraries(image_out ${catkin_LIBRARIES})
find_package(OpenCV REQUIRED)
target_link_libraries(image_out ${OpenCV_LIBS})
add_dependencies(image_out image_out_generate_message_cpp)

add_executable(image_out_test src/image_out_test.cpp)
target_link_libraries(image_out_test ${catkin_LIBRARIES})
find_package(OpenCV REQUIRED)
target_link_libraries(image_out_test ${OpenCV_LIBS})
add_dependencies(image_out_test image_out_test_generate_message_cpp)
