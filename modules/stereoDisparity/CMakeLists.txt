# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(stereoDisparity)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})


add_executable(stereoDisparity ${source} ${header})

target_link_libraries(stereoDisparity stereoVision iKin icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS stereoDisparity DESTINATION bin)

