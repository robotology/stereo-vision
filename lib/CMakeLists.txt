# Copyright: (C) 2011 RobotCub Consortium
# Author: Sean Ryan Fanello, Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME stereoVision)
project(${PROJECTNAME})

set(USE_SIFT_GPU ${IS_SIFT_GPU} CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

if(USE_SIFT_GPU)
    find_package(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories(${SIFTGPU_INCLUDE_DIRS})
    endif()
endif()

if(USE_SIFT_GPU)
    add_definitions(-DUSING_GPU)
endif()

############## LIBELAS ##############

# use sse3 instruction set
set (CMAKE_CXX_FLAGS "-msse3")

if (UNIX)
  find_package(OpenMP)
endif()

if(OPENMP_FOUND)

  message(STATUS "OPENMP FOUND")

  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

  set (LIBELAS_SRC_DIR elas_src_omp)

else()

  message(STATUS "OPENMP NOT FOUND")

  set (LIBELAS_SRC_DIR elas_src)
    
endif()

set (folder_header_elas 
                  ${LIBELAS_SRC_DIR}/elas.h
                  ${LIBELAS_SRC_DIR}/descriptor.h
                  ${LIBELAS_SRC_DIR}/image.h
                  ${LIBELAS_SRC_DIR}/filter.h
                  ${LIBELAS_SRC_DIR}/timer.h
                  ${LIBELAS_SRC_DIR}/matrix.h
                  ${LIBELAS_SRC_DIR}/triangle.h)

set (folder_source_elas
                  ${LIBELAS_SRC_DIR}/elas.cpp
                  ${LIBELAS_SRC_DIR}/descriptor.cpp
                  ${LIBELAS_SRC_DIR}/filter.cpp
                  ${LIBELAS_SRC_DIR}/matrix.cpp
                  ${LIBELAS_SRC_DIR}/triangle.cpp)

####################################

set(folder_source src/camera.cpp
		  src/stereoCamera.cpp
		  src/disparityThread.cpp
		  src/opticalFlowThread.cpp
		  src/sceneFlow.cpp
      src/elasWrapper.cpp)

set(folder_header include/iCub/stereoVision/camera.h
		  include/iCub/stereoVision/stereoCamera.h
		  include/iCub/stereoVision/disparityThread.h
		  include/iCub/stereoVision/opticalFlowThread.h
		  include/iCub/stereoVision/sceneFlow.h
      include/iCub/stereoVision/elasWrapper.h)


if(USE_SIFT_GPU)
    set(folder_source src/camera.cpp
		              src/stereoCamera.cpp
		              src/disparityThread.cpp
		              src/opticalFlowThread.cpp
		              src/sceneFlow.cpp
                  src/elasWrapper.cpp
                  src/utils.cpp)

    set(folder_header include/iCub/stereoVision/camera.h
	   	              include/iCub/stereoVision/stereoCamera.h
		              include/iCub/stereoVision/disparityThread.h
		              include/iCub/stereoVision/opticalFlowThread.h
		              include/iCub/stereoVision/sceneFlow.h
                  include/iCub/stereoVision/elasWrapper.h
                  include/iCub/stereoVision/utils.h)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/${LIBELAS_SRC_DIR})

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

source_group("ELAS Source Files" FILES ${folder_source_elas})
source_group("ELAS Header Files" FILES ${folder_header_elas})

add_library(${PROJECTNAME} ${folder_source} ${folder_header} ${folder_source_elas} ${folder_header_elas})

if(USE_SIFT_GPU)
    target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} iKin ${YARP_LIBRARIES} ${SIFTGPU_LIBRARIES}  )
else()
    target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} iKin ${YARP_LIBRARIES})
endif()

if(OPENMP_FOUND)

   target_link_libraries(${PROJECTNAME} ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS})

endif()

if(USE_SIFT_GPU)
icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/${LIBELAS_SRC_DIR}
                                          EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${SIFTGPU_INCLUDE_DIRS}
                                          DESTINATION include/iCub/${PROJECTNAME}
                                          FILES ${folder_header})
else()
icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/${LIBELAS_SRC_DIR}
                                          EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES}
                                          DESTINATION include/iCub/${PROJECTNAME}
                                          FILES ${folder_header})
endif()
