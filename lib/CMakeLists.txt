# Copyright: (C) 2011 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME stereoVision)
project(${PROJECTNAME})

set(USE_SIFT_GPU ${IS_SIFT_GPU} CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

if(OpenCV_VERSION_MINOR GREATER 2)
	add_definitions(-DUSE_NEWFEATURES)
	message(STATUS "OpenCV Minor is " ${OpenCV_VERSION_MINOR}", Using new Feature Structures")
endif()

if (USE_SIFT_GPU)
    find_package(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories( ${SIFTGPU_INCLUDE_DIRS})
    endif()
endif()

if(USE_SIFT_GPU)
    add_definitions(-DUSING_GPU)
endif()

set(folder_source src/camera.cpp
		  src/stereoCamera.cpp
		  src/disparityThread.cpp
		  src/opticalFlowThread.cpp
		  src/sceneFlow.cpp)

set(folder_header include/iCub/stereoVision/camera.h
		  include/iCub/stereoVision/stereoCamera.h
		  include/iCub/stereoVision/disparityThread.h
		  include/iCub/stereoVision/opticalFlowThread.h
		  include/iCub/stereoVision/sceneFlow.h)


if(USE_SIFT_GPU)
    set(folder_source src/camera.cpp
		              src/stereoCamera.cpp
		              src/disparityThread.cpp
		              src/opticalFlowThread.cpp
		              src/sceneFlow.cpp
                      src/utils.cpp)

    set(folder_header include/iCub/stereoVision/camera.h
	   	              include/iCub/stereoVision/stereoCamera.h
		              include/iCub/stereoVision/disparityThread.h
		              include/iCub/stereoVision/opticalFlowThread.h
		              include/iCub/stereoVision/sceneFlow.h
                      include/iCub/stereoVision/utils.h)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})
if(USE_SIFT_GPU)
    target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} iKin icubmod ${YARP_LIBRARIES} ${SIFTGPU_LIBRARIES}  )
else()
    target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} iKin icubmod ${YARP_LIBRARIES})
endif()

if(USE_SIFT_GPU)
icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                          EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${SIFTGPU_INCLUDE_DIRS}
                                          DESTINATION include/iCub/stereoVision
                                          FILES ${folder_header})
else()
icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                          EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES}
                                          DESTINATION include/iCub/stereoVision
                                          FILES ${folder_header})
endif()
