cmake_minimum_required(VERSION 2.8.3)

project(custom_msgs)

# Add support for c++11
add_definitions(-std=c++11 -O3)


# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
	FILES 
	FootInputMsg.msg
	FootOutputMsg.msg
	FeetOutputMsg.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(feet_output_publisher src/feet_output_publisher.cpp)
target_link_libraries(feet_output_publisher ${catkin_LIBRARIES})
add_dependencies(feet_output_publisher ${PROJECT_NAME}_gencpp)
