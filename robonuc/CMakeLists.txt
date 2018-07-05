cmake_minimum_required(VERSION 2.8.3)
project(robonuc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  dynamixel_msgs
  geometry_msgs
#  joy
  nav_msgs
  roscpp
  rospy
  genmsg
  sensor_msgs
  std_msgs
  tf
  sensor_msgs
  cv_bridge
  roscpp
  image_transport
  moveit_ros_planning_interface
  actionlib
  actionlib_msgs
  message_runtime
  message_generation
  r_platform
)

## System dependencies are found with CMake's conventions
# find_package(PCL REQUIRED)
# include_directories(
#  ${PCL_INCLUDE_DIRS}
#)
#link_directories(
#  ${PCL_LIBRARY_DIRS}
#)
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

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
	conjunto.msg
	teclado.msg
	robonuc_motion.msg
	ios.msg
	platmov.msg
	platpos.msg
	alarm.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
      robonuc_com.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
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
#  LIBRARIES segundo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS roscpp
   roscpp 
#message_runtime
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
# add_library(chapter4
#   src/${PROJECT_NAME}/chapter4.cpp
# )

## Declare a cpp executable
add_executable(vs_fanuc_client src/vs_fanuc_client.cpp)
add_executable(vs_keyboard src/vs_keyboard.cpp)
add_executable(vs_platform_sim src/vs_platform_sim.cpp)
add_executable(vs_IO_client src/vs_IO_client.cpp)
add_executable(vs_frame src/vs_frame.cpp)
add_executable(vs_laser_receptor src/vs_laser_receptor.cpp)
#add_executable(pcl_filtery_rgb src/pcl_filtery_rgb.cpp)
#add_executable(pcl_partitioning_rgb src/pcl_partitioning_rgb.cpp)
#add_executable(pcl_planar_segmentation_rgb src/pcl_planar_segmentation_rgb.cpp)
#add_executable(pcl_planar_segmentationTabuleiro_rgb src/pcl_planar_segmentationTabuleiro_rgb.cpp)
#add_executable(pcl_filter1 src/pcl_filter1.cpp)
#add_executable(pcl_save src/pcl_save.cpp)
#add_executable(pcl_planar_segmentationPecas_rgb src/pcl_planar_segmentationPecas_rgb.cpp)
#add_executable(pcl_centroides src/pcl_centroides.cpp)
#add_executable(pcl_convert src/pcl_convert.cpp)

# Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(chapte0r4_node chapter4_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(pcl_checkerboard_segmentation
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
# target_link_libraries(c4_example2
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# target_link_libraries(c4_example3
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

#target_link_libraries(pcl_visualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Declare a cpp executable
#add_executable(service_client src/service_client.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vs_keyboard ${catkin_LIBRARIES})
target_link_libraries(vs_platform_sim ${catkin_LIBRARIES})
target_link_libraries(vs_fanuc_client ${catkin_LIBRARIES})
target_link_libraries(vs_IO_client ${catkin_LIBRARIES})
target_link_libraries(vs_frame ${catkin_LIBRARIES})
target_link_libraries(vs_laser_receptor ${catkin_LIBRARIES})

# target_link_libraries(service_client ${catkin_LIBRARIES})
# target_link_libraries(main ${catkin_LIBRARIES


#  add_dependencies(service_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(vs_keyboard ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(vs_platform_sim ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(vs_platform_sim ${PROJECT_NAME}_gencpp)
add_dependencies(vs_fanuc_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(vs_fanuc_client ${PROJECT_NAME}_gencpp)
add_dependencies(vs_laser_receptor ${PROJECT_NAME}_gencpp)
add_dependencies(vs_platform_sim ${catkin_EXPORTED_TARGETS})


#add_dependencies(main ${PROJECT_NAME}_generate_messages_cpp)

#target_link_libraries(pcl_filtery_rgb ${catkin_LIBRARIES} ${PCL_LIBRARIES})
##target_link_libraries(pcl_partitioning_rgb ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_planar_segmentation_rgb ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_planar_segmentationTabuleiro_rgb ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_filter1 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_save ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_planar_segmentationPecas_rgb ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_centroides ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(pcl_convert ${catkin_LIBRARIES} ${PCL_LIBRARIES})
## target_link_libraries(c4_example4
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# target_link_libraries(c4_example5_2
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# target_link_libraries(c4_example6
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# target_link_libraries(c4_example7
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# target_link_libraries(c4_example8
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )
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
# install(TARGETS chapter4 chapter4_node
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
# Mark executables and/or libraries for installation
# install(TARGETS service_client service_server
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
#)
# Mark executables and/or libraries for installation
install(TARGETS vs_fanuc_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chapter4.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
