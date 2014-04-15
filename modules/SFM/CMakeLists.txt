# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

if (USE_SIFT_GPU)
    find_package(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories( ${SIFTGPU_INCLUDE_DIRS})
    endif()
endif()

if (USE_SIFT_GPU)
    ADD_DEFINITIONS(-DUSING_GPU)
endif()

set(source SFM_module.cpp)
set(header SFM_module.h)

#if (USE_SIFT_GPU)
#    set(source ${source} utils.cpp)
#    set(header ${header} utils.h)
#endif()

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
add_executable(SFM ${source} ${header})

if (USE_SIFT_GPU)
	target_link_libraries(SFM stereoVision iKin icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${SIFTGPU_LIBRARIES})
else()
	target_link_libraries(SFM stereoVision iKin icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
endif()

install(TARGETS SFM DESTINATION bin)
