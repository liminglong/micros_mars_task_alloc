cmake_minimum_required(VERSION 2.8.3)
project(micros_mars_task_alloc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  kobuki_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  cv_bridge
  nav_msgs
  nodelet
  message_filters
)



find_package(Boost REQUIRED)
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
 add_message_files(
   DIRECTORY
   msg
   FILES
   Heartbeat.msg
   Force.msg
   Heading.msg
   Path.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY
   action
   FILES
   MoveBase.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs   geometry_msgs  std_msgs sensor_msgs 
 )
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES micros_mars_task_alloc
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation roscpp rospy std_msgs kobuki_msgs sensor_msgs cv_bridge nav_msgs message_filters
  DEPENDS system_lib Boost
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  SYSTEM ${BOOST_INCLUDE_DIRS}
         ${catkin_INCLUDE_DIRS}

)

## Declare a C++ library
add_library(${PROJECT_NAME}
#APP1: The three-layer autonomous wander robot based on subsumption model.
    #layer 0 of the autonomous wander robot
    src/nodelets/auto_wander_robot/layer0/feelforce.cpp
    src/nodelets/auto_wander_robot/layer0/runaway.cpp
    src/nodelets/auto_wander_robot/layer0/collide.cpp
    src/nodelets/auto_wander_robot/layer0/turn.cpp
    src/nodelets/auto_wander_robot/layer0/forward.cpp
    #layer 1 of the autonomous wander robot
    src/nodelets/auto_wander_robot/layer1/wander.cpp
    src/nodelets/auto_wander_robot/layer1/avoid.cpp
    src/nodelets/auto_wander_robot/layer1/avoid_runaway_suppressor.cpp   
    #layer 2 of the autonomous wander robot
    src/nodelets/auto_wander_robot/layer2/status.cpp
    src/nodelets/auto_wander_robot/layer2/whenlook.cpp
    src/nodelets/auto_wander_robot/layer2/pathplan.cpp
    src/nodelets/auto_wander_robot/layer2/look.cpp
    src/nodelets/auto_wander_robot/layer2/whenlook_wander_inhibitor.cpp
    src/nodelets/auto_wander_robot/layer2/pathplan_wander_suppressor.cpp
   
#APP2: The multiple security patrol robots based on ALLIANCE model.
    src/nodelets/multi_patrol_robot/detect.cpp   
    src/nodelets/multi_patrol_robot/follow.cpp      
    src/nodelets/multi_patrol_robot/patrol.cpp
    
    src/nodelets/multi_patrol_robot/forwarder_bool.cpp
    src/nodelets/multi_patrol_robot/motivational_behavior_test.cpp
 )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(micros_mars_task_alloc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(micros_mars_task_alloc_node src/micros_mars_task_alloc_node.cpp)
#add_executable(shared_ptr_test src/nodes/shared_ptr_test.cpp)
add_executable(mf_test src/nodes/mf_test.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(micros_mars_task_alloc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(micros_mars_task_alloc_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(shared_ptr_test
#   ${catkin_LIBRARIES}
# )
 target_link_libraries(mf_test
   ${catkin_LIBRARIES}
 )
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
# install(TARGETS micros_mars_task_alloc micros_mars_task_alloc_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_micros_mars_task_alloc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
