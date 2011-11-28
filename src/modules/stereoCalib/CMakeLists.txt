# Copyright: (C) 2010 RobotCub Consortium
# Authors: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

project(stereoCalib)


file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})



add_executable(stereoCalib ${source} ${header})
target_link_libraries(stereoCalib stereoVision ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS stereoCalib DESTINATION ${ICUB_DIR}/bin)

