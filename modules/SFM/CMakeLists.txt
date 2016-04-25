# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)

set(source SFM.cpp)
set(header SFM.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${stereoVision_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
    add_definitions(-DUSING_GPU)
endif()

add_executable(${PROJECT_NAME} ${source} ${header})
target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

if(USE_SIFT_GPU)
	target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
