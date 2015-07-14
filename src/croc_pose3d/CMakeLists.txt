cmake_minimum_required(VERSION 2.8.3)
project(croc_pose3d)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  geometry_msgs
  cyphy_serial_driver
  tf
)

add_message_files(FILES
  Pose3D.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp sensor_msgs cyphy_serial_driver geometry_msgs tf
  INCLUDE_DIRS include
#  LIBRARIES cyphy_serial_driver
)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(croc_pose3d src/croc_Pose3D.cpp)
add_dependencies(croc_pose3d ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(croc_pose3d ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_testpkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
