cmake_minimum_required(VERSION 2.8.3)
project(ndt_test)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -fPIC -Wno-sign-compare -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -O3)

include(DistVersion.cmake)
system_info(DISTRO)
message(STATUS "DISTRO: ${DISTRO}")

add_subdirectory(include/libicp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  ifopt
)

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${PROJECT_SOURCE_DIR}/cmake_modules"
)

find_package(PCL REQUIRED)
find_package(CUDA)
find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${Boost_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

## octomap 
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

# if(${DISTRO} MATCHES "14.04")
#   find_package(Qt4 REQUIRED COMPONENTS QtCore QtXml QtOpenGL QtGui QtMultimedia)
#   include(${QT_USE_FILE})
#   include_directories(${QT_INCLUDES})
#   set(MY_QT_LIBRARIES
#       Qt4::QtCore
#       Qt4::QtXml
#       Qt4::QtOpenGL
#       Qt4::QtGui)
# elseif(${DISTRO} MATCHES "16.04" OR ${DISTRO} MATCHES "18.04")
#   find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)
#   include_directories(${Qt5Core_INCLUDE_DIRS}
#                       ${Qt5Xml_INCLUDE_DIRS}
#                       ${Qt5Gui_INCLUDE_DIRS}
#                       ${Qt5Widgets_INCLUDE_DIRS}
#                       ${Qt5OpenGL_INCLUDE_DIRS})
#   set(MY_QT_LIBRARIES
#       ${Qt5Widgets_LIBRARIES}
#       ${Qt5Core_LIBRARIES}
#       ${Qt5Gui_LIBRARIES}
#       ${Qt5Xml_LIBRARIES}
#       ${Qt5OpenGL_LIBRARIES})
# endif()

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
#   std_msgs
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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_test
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_definitions(-DNDT_USE_GPU)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIR}
  ../ndt_gpu/include
)

## build some libs for depth clustering

set(DEPTH_CLUSTER_DIR include/depth_clustering)
include_directories(${DEPTH_CLUSTER_DIR})

add_library(projections SHARED
  ${DEPTH_CLUSTER_DIR}/projections/projection_params.cpp
  ${DEPTH_CLUSTER_DIR}/projections/cloud_projection.cpp
  ${DEPTH_CLUSTER_DIR}/projections/ring_projection.cpp
  ${DEPTH_CLUSTER_DIR}/projections/spherical_projection.cpp)
target_link_libraries(projections 
  ${OpenCV_LIBS})

add_library(identifiable SHARED ${DEPTH_CLUSTER_DIR}/communication/identifiable.cpp)
add_library(ground_remove SHARED ${DEPTH_CLUSTER_DIR}/ground_removal/depth_ground_remover.cpp)
add_library(difference SHARED
            ${DEPTH_CLUSTER_DIR}/image_labelers/diff_helpers/angle_diff.cpp
            ${DEPTH_CLUSTER_DIR}/image_labelers/diff_helpers/line_dist_diff.cpp)
target_link_libraries(ground_remove
                      identifiable
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})
                      
add_library(cloud SHARED 
  ${DEPTH_CLUSTER_DIR}/utils/cloud.cpp ${DEPTH_CLUSTER_DIR}/utils/rich_point.cpp
  ${DEPTH_CLUSTER_DIR}/utils/cloud_saver.cpp)
target_link_libraries(cloud
  projections ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(bbox 
  ${DEPTH_CLUSTER_DIR}/utils/bbox.cpp)
target_link_libraries(bbox 
  cloud)
  
# find_package(QGLViewer REQUIRED)
# 
# add_library(visualization SHARED 
#   ${DEPTH_CLUSTER_DIR}/visualization/visualizer.cpp)
# target_link_libraries(visualization
#                       identifiable
#                       ${QGLVIEWER_LIBRARY}
#                       ${OPENGL_gl_LIBRARY}
#                       ${MY_QT_LIBRARIES}
#                       ${OPENGL_glu_LIBRARY})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# set(srcs
#   src/main.cpp 
#   src/macro_defines.cc
#   src/map_builder.cc
#   src/msg_conversion.cc
#   src/pose_graph.cc
#   src/multiview_icp_ceres.cc )

file(GLOB common_srcs "src/common/*.cc")
file(GLOB srcs "src/*.cc")
add_executable(${PROJECT_NAME}_node 
  ${srcs} ${common_srcs})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
set(depth_clustring_libs 
  projections ground_remove cloud)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  icp
  ceres
  gtsam
  ndt_gpu
  ${depth_clustring_libs}
  ${OCTOMAP_LIBRARIES}
)
