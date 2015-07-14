cmake_minimum_required(VERSION 2.8.3)
project(croc_calc_landing_field_pos)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	croc_pose3d
	geometry_msgs
  	sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp croc_pose3d geometry_msgs sensor_msgs
#    INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(croc_CalcLandingPos src/main.cpp)
target_link_libraries(croc_CalcLandingPos ${catkin_LIBRARIES})



##FIND_PACKAGE(Qt4)

#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})
#TARGET_LINK_LIBRARIES(croc_CalcLandingPos ${QT_LIBRARIES} )
