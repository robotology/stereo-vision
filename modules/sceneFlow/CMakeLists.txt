# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(sceneFlowModule)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${stereoVision_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})
add_executable(sceneFlowModule ${source} ${header})
target_link_libraries(sceneFlowModule stereoVision ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS sceneFlowModule DESTINATION bin)
