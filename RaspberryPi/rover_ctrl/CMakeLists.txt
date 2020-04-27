cmake_minimum_required(VERSION 2.8.3)
project(rover_ctrl)

add_compile_options( -std=c++0x )

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES Rov_ctrl.msg Joints_info.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build the nodes
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(nav_node src/nav_node.cpp)
target_link_libraries(nav_node ${catkin_LIBRARIES})
add_dependencies(nav_node rover_ctrl_generate_messages_cpp)

add_executable(read_state src/read_state.cpp)
target_link_libraries(read_state ${catkin_LIBRARIES})

add_executable(cmd_mt src/cmd_mt.cpp src/ModelTree/cpp/model_tree.cc) # git clone https://github.com/Bouty92/ModelTree
target_link_libraries(cmd_mt ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(cmd_mt rover_ctrl_generate_messages_cpp)
