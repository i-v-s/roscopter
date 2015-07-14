cmake_minimum_required(VERSION 2.8.3)
project(cyphy_serial_driver)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  sensor_msgs 
  control_toolbox 
  std_msgs 
  genmsg
)

###############
## Pre Build ##
###############

add_message_files(FILES
  mikoCmd.msg
  mikoImu.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

############
## Export ##
############

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs control_toolbox
    INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS}, ${catkin_INCLUDE_DIRS})

add_executable(MikoControl src/flightcontrol.cpp src/serial_interface.cpp)
add_dependencies(MikoControl ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(MikoControl ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#   scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
# install(TARGETS testpkg testpkg_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

