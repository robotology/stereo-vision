# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)

if(USE_SIFT_GPU)
    find_package(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories(${SIFTGPU_INCLUDE_DIRS})
        add_definitions(-DUSING_GPU)
    endif()
endif()

set(source SFM.cpp)
set(header SFM.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${stereoVision_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})
add_executable(SFM ${source} ${header})
target_link_libraries(SFM stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

if(USE_SIFT_GPU)
	target_link_libraries(SFM ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS SFM DESTINATION bin)
