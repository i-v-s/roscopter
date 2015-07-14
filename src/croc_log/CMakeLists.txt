cmake_minimum_required(VERSION 2.8.3)
project(croc_log)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	croc_pose3d
	croc_command
	cyphy_serial_driver
	px_comm
)

catkin_package(
    CATKIN_DEPENDS roscpp croc_pose3d croc_command cyphy_serial_driver px_comm
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(croc_log src/croc_log.cpp)
target_link_libraries(croc_log ${catkin_LIBRARIES})


