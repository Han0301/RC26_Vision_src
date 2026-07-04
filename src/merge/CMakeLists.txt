#cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 2.8.3)
project(merge)



SET(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")

ADD_COMPILE_OPTIONS(-std=c++17)
ADD_COMPILE_OPTIONS(-std=c++17)
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()










## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(teaserpp REQUIRED)
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
# 强制跳过 fastcdr/fastrtps 依赖检查
set(realsense2_FOUND TRUE)

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
include_directories(/usr/include/libusb-1.0)
link_directories(/usr/lib/x86_64-linux-gnu)
#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  tf
  tf2
  tf2_msgs
  tf2_ros
  vision_msgs
  rosbag
  livox_ros_driver
  livox_ros_driver2
)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
#find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(PkgConfig REQUIRED)

find_package(realsense2)

pkg_check_modules(LIBSERIAL REQUIRED libserial)
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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   tf2_msgs#   vision_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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




find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
message(STATUS "Found OpenVINO v${OpenVINO_VERSION}")
message(STATUS "OpenVINO include dir: ${OpenVINO_INCLUDE_DIRS}")
message(STATUS "OpenVINO libraries: ${OpenVINO_LIBRARIES}")
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge vision_msgs
  DEPENDS OpenVINO
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES merge
#  CATKIN_DEPENDS camera_info_manager cv_bridge geometry_msgs image_geometry image_transport livox_ros_driver2 nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs serial std_msgs tf tf2 tf2_msgs tf2_ros vision_msgs
#  DEPENDS system_lib
#)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_INCLUDE_DIRS}
  ${LIBSERIAL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TEASERPP_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${TEASERPP_LIBRARY_DIRS}
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/merge.cpp
# )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/merge_node.cpp)
add_executable(merge_node src/merge.cpp)
#add_executable(merge2_node src/merge2.cpp)
# add_executable(livox_ros_driver2_node src/livox_ros_driver2/src/livox_ros_driver2.cpp)
# add_executable(pointlio_mapping src/point_lio/src/laserMapping.cpp 
#                 src/point_lio/src/li_initialization.cpp src/point_lio/src/parameters.cpp src/point_lio/src/preprocess.cpp src/point_lio/src/Estimator.cpp 
#                 src/point_lio/src/IMU_Processing.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


# target_sources(livox_ros_driver2_node
#   PRIVATE
#   src/livox_ros_driver2/src/driver_node.cpp
#   src/livox_ros_driver2/src/lds.cpp
#   src/livox_ros_driver2/src/lds_lidar.cpp
#   src/livox_ros_driver2/src/lddc.cpp
#   src/livox_ros_driver2/src/livox_ros_driver2.cpp

#   src/livox_ros_driver2/src/comm/comm.cpp
#   src/livox_ros_driver2/src/comm/ldq.cpp
#   src/livox_ros_driver2/src/comm/semaphore.cpp
#   src/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp
#   src/livox_ros_driver2/src/comm/cache_index.cpp
#   src/livox_ros_driver2/src/comm/pub_handler.cpp

#   src/livox_ros_driver2/src/parse_cfg_file/parse_cfg_file.cpp
#   src/livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp

#   src/livox_ros_driver2/src/call_back/lidar_common_callback.cpp
#   src/livox_ros_driver2/src/call_back/livox_lidar_callback.cpp


# )


find_library(URCU_LIB NAMES urcu urcu-memb)
if(NOT URCU_LIB)
  message(FATAL_ERROR "liburcu not found! Please install liburcu-dev.")
endif()


target_sources(merge_node
  PRIVATE
  src/lidar.cpp
  src/method_math.cpp
  src/openvino.cpp
  src/serial.cpp
  src/camera.cpp
  src/coordinate.cpp
  src/velocity.cpp
  src/calibration.cpp
  src/threadpool.cpp
  src/log/logger.cpp
  src/recognition/world_to_camera.cpp
  src/recognition/zbuffer_simplify.cpp
  src/recognition/occlusion_handing.cpp

  src/livox_ros_driver2/src/driver_node.cpp
  src/livox_ros_driver2/src/lds.cpp
  src/livox_ros_driver2/src/lds_lidar.cpp
  src/livox_ros_driver2/src/lddc.cpp
  #src/livox_ros_driver2/src/livox_ros_driver2.cpp
  src/livox_ros_driver2/src/livox_ros_driver.cpp
  src/livox_ros_driver2/src/comm/comm.cpp
  src/livox_ros_driver2/src/comm/ldq.cpp
  src/livox_ros_driver2/src/comm/semaphore.cpp
  src/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp
  src/livox_ros_driver2/src/comm/cache_index.cpp
  src/livox_ros_driver2/src/comm/pub_handler.cpp

  src/livox_ros_driver2/src/parse_cfg_file/parse_cfg_file.cpp
  src/livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/livox_ros_driver2/src/call_back/lidar_common_callback.cpp
  src/livox_ros_driver2/src/call_back/livox_lidar_callback.cpp

  src/point_lio/src/laserMapping2.cpp
  src/point_lio/src/li_initialization.cpp 
  src/point_lio/src/parameters.cpp 
  src/point_lio/src/preprocess.cpp 
  src/point_lio/src/Estimator.cpp 
  src/point_lio/src/IMU_Processing.cpp
)

# target_sources(merge2_node
#   PRIVATE
#   src/lidar.cpp
#   src/livox_ros_driver2/src/driver_node.cpp
#   src/livox_ros_driver2/src/lds.cpp
#   src/livox_ros_driver2/src/lds_lidar.cpp
#   src/livox_ros_driver2/src/lddc.cpp
#   #src/livox_ros_driver2/src/livox_ros_driver2.cpp
#   #src/livox_ros_driver2/src/livox_ros_driver.cpp
#   src/livox_ros_driver2/src/comm/comm.cpp
#   src/livox_ros_driver2/src/comm/ldq.cpp
#   src/livox_ros_driver2/src/comm/semaphore.cpp
#   src/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp
#   src/livox_ros_driver2/src/comm/cache_index.cpp
#   src/livox_ros_driver2/src/comm/pub_handler.cpp

#   src/livox_ros_driver2/src/parse_cfg_file/parse_cfg_file.cpp
#   src/livox_ros_driver2/src/parse_cfg_file/parse_livox_lidar_cfg.cpp

#   src/livox_ros_driver2/src/call_back/lidar_common_callback.cpp
#   src/livox_ros_driver2/src/call_back/livox_lidar_callback.cpp

#   #src/point_lio/src/laserMapping2.cpp
#   src/point_lio/src/li_initialization.cpp 
#   src/point_lio/src/parameters.cpp 
#   src/point_lio/src/preprocess.cpp 
#   src/point_lio/src/Estimator.cpp 
#   src/point_lio/src/IMU_Processing.cpp
# )

target_link_libraries(merge_node
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${Boost_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${URCU_LIB}
  #${realsense2_LIBRARIES} 
  /usr/lib/x86_64-linux-gnu/librealsense2.so
  openvino::runtime
  teaser_registration
  teaser_io
)


# target_link_libraries(merge2_node
#   ${LIVOX_LIDAR_SDK_LIBRARY}
#   ${Boost_LIBRARY}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${APR_LIBRARIES}
#   ${PYTHON_LIBRARIES}
#   ${URCU_LIB}
#   #${realsense2_LIBRARIES} 
#   /usr/lib/x86_64-linux-gnu/librealsense2.so
#   openvino::runtime
# )

target_include_directories(merge_node PRIVATE ${PYTHON_INCLUDE_DIRS})
# target_include_directories(merge2_node PRIVATE ${PYTHON_INCLUDE_DIRS})

# target_link_libraries(livox_ros_driver2_node
#   ${LIVOX_LIDAR_SDK_LIBRARY}
#   ${Boost_LIBRARY}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${APR_LIBRARIES}
# )
# target_link_libraries(pointlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
# target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_merge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
