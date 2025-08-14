cmake_minimum_required(VERSION 3.0.2)
project(tta_m3e_rtsp)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
message("")
message("")
message("===================Start found package==========================")
set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)

# solve HSV location
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message(STATUS "OpenCV found: ${OpenCV_VERSION}")
  message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OpenCV library path: ${OpenCV_LIBRARY_DIRS}")
  message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")
else()
  message(FATAL_ERROR "OpenCV not found. Please Stop the process and download!")
endif()

# use pkg-config findout ffmpeg
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED
  libavutil
  libavcodec
  libavformat
  libavdevice
  libswscale
  libswresample
  libpostproc
)

if(FFMPEG_FOUND)
  message(STATUS "FFmpeg found: ${FFMPEG_VERSION}")
  message(STATUS "FFmpeg include path: ${FFMPEG_INCLUDE_DIRS}")
  message(STATUS "FFmpeg library path: ${FFMPEG_LIBRARY_DIRS}")
  message(STATUS "FFmpeg libraries: ${FFMPEG_LIBRARIES}")
else()
  message(FATAL_ERROR "ffmpeg not found. Please Stop the process and checkout tta's tutorials!")
endif()

find_package(ZLIB REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  uavdata.msg
  flightByVel.msg
)
## Generate services in the 'srv' folder
add_service_files(
  FILES
  yolov5Result.srv
  gimbalControl.srv
  takeoffOrLanding.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  
)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime message_generation nav_msgs geometry_msgs dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
  ${ZLIB_INCLUDE_DIRS}
  src/Controller/utils
  src/Controller/proxy_src
  src/Controller/ttalink
)

############################################################################
##                      Create dynamic libraries                          ##
#############################################################################

aux_source_directory(src/Controller/proxy_src PROXY_SRCS)
add_library(proxy_lib ${PROXY_SRCS})

aux_source_directory(src/Controller/utils UTILS_SRCS)
add_library(utils_lib ${UTILS_SRCS})


## To solve the errors that occur when installing third - party libraries.
# The FFmpeg, MPP and other software we installed are all placed here.
link_directories(/usr/local/lib)
set(RGA_LIB /usr/lib/librga.so)
set(MPP_LIB /usr/lib/aarch64-linux-gnu/librockchip_mpp.so)

########################################################################
##                    Create executable file                          ##
########################################################################
## Add source files for RTSP video stream processing.

## controller plane
add_executable(fcProxy src/Controller/fcProxy.cpp src/Controller/uavData.cpp src/Controller/publish.cpp)
target_link_libraries(fcProxy
  ${catkin_LIBRARIES}
  ${RPI_LIBRARIES}
  ${ZLIB_LIBRARIES}
  proxy_lib
  utils_lib
)

## uav odom
add_executable(uavNav src/uav_odom.cpp)
target_link_libraries(uavNav
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

