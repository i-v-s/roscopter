cmake_minimum_required(VERSION 2.8.3)
project(croc_command)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	control_toolbox
	croc_pose3d
	geometry_msgs
	message_generation
)

add_message_files(FILES
	PID.msg
	State.msg
	mikoCmd.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp croc_pose3d control_toolbox geometry_msgs
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(croc_command src/croc_command.cpp)
add_dependencies(croc_command ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(croc_command ${catkin_LIBRARIES})


