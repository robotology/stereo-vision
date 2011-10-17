# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME stereoModules)

project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)
find_package(stereoVision)

SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${stereoVision_INCLUDE_DIRS})

INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

add_subdirectory(stereoDisparity)
add_subdirectory(stereoCalib)


