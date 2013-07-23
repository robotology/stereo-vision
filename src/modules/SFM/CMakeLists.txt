# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
SET(USE_SIFT_GPU ${IS_SIFT_GPU} CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")

if (USE_SIFT_GPU)
    FIND_PACKAGE(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories( ${SIFTGPU_INCLUDE_DIRS})
    endif()
endif()

if (USE_SIFT_GPU)
    ADD_DEFINITIONS(-DUSING_GPU)
ENDIF()

file(GLOB source SFM_module.cpp)
file(GLOB header SFM_module.h)

if (USE_SIFT_GPU)
    file(GLOB source SFM_module.cpp utils.cpp)
    file(GLOB header SFM_module.h utils.h)
ENDIF()

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

add_executable(SFM ${source} ${header} )

    if (USE_SIFT_GPU)
        target_link_libraries(SFM stereoVision iKin icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${SIFTGPU_LIBRARIES} )
    else ()
        target_link_libraries(SFM stereoVision iKin icubmod ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
    endif()

install(TARGETS SFM DESTINATION ${ICUB_DIR}/bin)

