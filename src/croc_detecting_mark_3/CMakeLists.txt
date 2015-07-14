cmake_minimum_required(VERSION 2.8.3)
project(croc_detecting_mark_3)

#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	image_transport
	cv_bridge
)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_message_files(FILES
	HSVrange.msg
)

generate_messages()

catkin_package(
#    DEPENDS roscpp sensor_msgs control_toolbox
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include
#    LIBRARIES
)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS}, ${catkin_INCLUDE_DIRS}, ${OpenCV2_INCLUDE_DIRS})

#DetectingMark
add_executable(croc_detecting_mark_3 src/main.cpp src/detecting_mark.cpp)
add_dependencies(croc_detecting_mark_3 ${PROJECT_NAME}_generate_messages_cpp)
#FIND_PACKAGE(Qt4)

#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})

find_package(OpenCV REQUIRED)
#include_directories(${QT_USE_FILE}  ${OpenCV2_INCLUDE_DIRS})
#include_directories(${OpenCV2_INCLUDE_DIRS})

target_link_libraries(croc_detecting_mark_3
	${catkin_LIBRARIES}
	opencv_core
	opencv_imgproc
	opencv_highgui
	opencv_ml
	opencv_video
	opencv_features2d
	opencv_calib3d
	opencv_objdetect
	opencv_contrib
	opencv_legacy
	opencv_flann
)
