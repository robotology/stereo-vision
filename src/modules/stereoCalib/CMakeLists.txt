# Copyright: (C) 2010 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

project(stereoCalib)
find_package(YARP)
find_package(ICUB)

SET(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

INCLUDE(iCubOptions)
INCLUDE(iCubHelpers)

find_package(OpenCV)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})


add_executable(stereoCalib ${source} ${header})
target_link_libraries(stereoCalib ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS stereoCalib DESTINATION ${ICUB_DIR}/bin)

