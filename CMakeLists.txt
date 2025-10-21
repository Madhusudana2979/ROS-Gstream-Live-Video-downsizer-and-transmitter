cmake_minimum_required(VERSION 3.10)
project(camera_publisher)

# Use C++11 standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin macros and ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# GStreamer dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_ALLOCATORS REQUIRED gstreamer-allocators-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

# CUDA runtime library only (no compiler needed)
find_package(CUDA REQUIRED)
# Don't enable CUDA language to avoid compiler issues

# Jetson Multimedia API paths
set(NVMMAPI_PATH /usr/src/jetson_multimedia_api)
set(NVMMAPI_INCLUDE_DIRS
  ${NVMMAPI_PATH}/include
  ${NVMMAPI_PATH}/include/utils
  ${NVMMAPI_PATH}/include/libjpeg-8b
  ${NVMMAPI_PATH}/samples/common/classes
)

# Explicit GStreamer include paths (ensure gstfdmemory.h is found)
include_directories(
  /usr/include/gstreamer-1.0
  /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${GSTREAMER_ALLOCATORS_INCLUDE_DIRS}
  ${GSTREAMER_VIDEO_INCLUDE_DIRS}
  ${NVMMAPI_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${CUDA_LIBRARY_DIRS}
  /usr/lib/aarch64-linux-gnu/tegra
  /usr/local/cuda/lib64
)

# Executable
add_executable(camera_publisher_node src/camera_publisher.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvJpegDecoder.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvBuffer.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvElement.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvElementProfiler.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvLogging.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvUtils.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvVideoEncoder.cpp 
  ${NVMMAPI_PATH}/samples/common/classes/NvV4l2Element.cpp
  ${NVMMAPI_PATH}/samples/common/classes/NvV4l2ElementPlane.cpp
)

# Link libraries - Updated with proper CUDA libraries
target_link_libraries(camera_publisher_node
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${GSTREAMER_ALLOCATORS_LIBRARIES}
  ${GSTREAMER_VIDEO_LIBRARIES}
  nvjpeg
  nvbuf_utils
  nvbufsurface
  nvbufsurftransform
  nvv4l2
  jpeg
  pthread
  dl
  ${CUDA_LIBRARIES}
  cudart
  cuda
)