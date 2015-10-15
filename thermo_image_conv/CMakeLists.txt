cmake_minimum_required(VERSION 2.8.3)
project(thermo_image_conv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
)

add_message_files(FILES TIconverter.msg)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thermo_image_conv_node src/thermo_image_conv.cpp)
target_link_libraries(thermo_image_conv_node ${catkin_LIBRARIES})
add_dependencies(thermo_image_conv_node thermo_image_conv_generate_message_cpp)

add_executable(thermo_image_conv_listener src/thermo_image_conv_listener.cpp)
target_link_libraries(thermo_image_conv_listener ${catkin_LIBRARIES})


add_executable(rotate_thermo_image_node src/rotate_thermo_image.cpp)
target_link_libraries(rotate_thermo_image_node ${catkin_LIBRARIES})
