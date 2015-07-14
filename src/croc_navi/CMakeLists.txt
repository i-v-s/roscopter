cmake_minimum_required(VERSION 2.8.3)
project(croc_navi)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	croc_pose3d
	croc_command
)

catkin_package(
    CATKIN_DEPENDS roscpp croc_pose3d croc_command
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(croc_navi src/croc_navi.cpp)
target_link_libraries(croc_navi ${catkin_LIBRARIES})


