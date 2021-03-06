cmake_minimum_required(VERSION 2.8.3)
project(bwi_scavenger)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  bwi_kr_execution
  pcl_ros
  cv_bridge
  image_transport
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES ColorShirt.srv FetchObject.srv Dialog.srv ObjectDetection.srv
  Whiteboard.srv TargetSearch.srv RedButton.srv RedButtonAction.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES VisionTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bwi_scavenger
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bwi_scavenger
#   src/${PROJECT_NAME}/bwi_scavenger.cpp
# )

## Declare a cpp executable

add_executable(blue_shirt src/blue_shirt.cpp)
target_link_libraries(blue_shirt ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(blue_shirt ${catkin_EXPORTED_TARGETS})
add_dependencies(blue_shirt ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(white_board src/white_board.cpp)
target_link_libraries(white_board ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(white_board ${catkin_EXPORTED_TARGETS})
add_dependencies(white_board ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(img_capture src/img_capture.cpp)
target_link_libraries(img_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(img_capture ${catkin_EXPORTED_TARGETS})
add_dependencies(img_capture ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(object_detection src/object_detection.cpp)
target_link_libraries(object_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(object_detection ${catkin_EXPORTED_TARGETS})
add_dependencies(object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(scav_hunt src/scav_hunt.cpp)
target_link_libraries(scav_hunt ${catkin_LIBRARIES})
add_dependencies(scav_hunt ${catkin_EXPORTED_TARGETS})
add_dependencies(scav_hunt ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(target_search src/target_search.cpp)
target_link_libraries(target_search ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(target_search ${catkin_EXPORTED_TARGETS})
add_dependencies(target_search ${${PROJECT_NAME}_EXPORTED_TARGETS})

# add_dependencies(vision_server)

add_executable(vision_server src/vision_server.cpp)
target_link_libraries(vision_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(vision_server ${catkin_EXPORTED_TARGETS})
add_dependencies(vision_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(red_button src/red_button.cpp)
target_link_libraries(red_button ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(red_button ${catkin_EXPORTED_TARGETS})
add_dependencies(red_button ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(red_button_action src/red_button_action.cpp)
target_link_libraries(red_button_action ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(red_button_action ${catkin_EXPORTED_TARGETS})
add_dependencies(red_button_action ${${PROJECT_NAME}_EXPORTED_TARGETS})


# add_dependencies(blue_shirt object_detection take_human_input white_board)
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
# install(TARGETS bwi_scavenger bwi_scavenger_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_bwi_scavenger.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
