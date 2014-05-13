# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(sceneFlowModule)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(sceneFlowModule ${source} ${header})
target_link_libraries(sceneFlowModule stereoVision ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS sceneFlowModule DESTINATION bin)

