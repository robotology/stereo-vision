# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})


add_executable(SFM ${source} ${header} )

target_link_libraries(SFM stereoVision iKin icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS SFM DESTINATION ${ICUB_DIR}/bin)

