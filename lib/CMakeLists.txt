# Copyright: (C) 2011 RobotCub Consortium
# Author: Sean Ryan Fanello, Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME stereoVision)
project(${PROJECTNAME})

set(USE_SIFT_GPU ${IS_SIFT_GPU} CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")
if(USE_SIFT_GPU)
    find_package(SIFTGPU)
    if(SIFTGPU_FOUND)
        include_directories(${SIFTGPU_INCLUDE_DIRS})
        add_definitions(-DUSING_GPU)
    endif()
endif()

############## LIBELAS ##############

list(APPEND CMAKE_CXX_FLAGS "-msse3")

if(UNIX)
    find_package(OpenMP)
endif()

if(OPENMP_FOUND)
  message(STATUS "OpenMP FOUND")

  list(APPEND CMAKE_C_FLAGS ${OpenMP_C_FLAGS})
  list(APPEND CMAKE_CXX_FLAGS ${OpenMP_CXX_FLAGS})

  set(folder_source_elas elas/src/elas_omp.cpp
                         elas/src/descriptor.cpp
                         elas/src/filter.cpp
                         elas/src/matrix.cpp
                         elas/src/triangle.cpp)
else()
  message(STATUS "OpenMP NOT FOUND")
  set(folder_source_elas elas/src/elas.cpp
                         elas/src/descriptor.cpp
                         elas/src/filter.cpp
                         elas/src/matrix.cpp
                         elas/src/triangle.cpp)
endif()

set(folder_header_elas elas/include/elas.h
                       elas/include/descriptor.h
                       elas/include/image.h
                       elas/include/filter.h
                       elas/include/timer.h
                       elas/include/matrix.h
                       elas/include/triangle.h)

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

if(SIFTGPU_FOUND)
    list(APPEND folder_source src/utils.cpp)
    list(APPEND folder_header include/iCub/stereoVision/utils.h)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/elas/include)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

source_group("ELAS Source Files" FILES ${folder_source_elas})
source_group("ELAS Header Files" FILES ${folder_header_elas})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})
add_library(${PROJECTNAME} ${folder_source} ${folder_header} ${folder_source_elas} ${folder_header_elas})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} iKin ${YARP_LIBRARIES})

if(SIFTGPU_FOUND)
    target_link_libraries(${PROJECTNAME} ${SIFTGPU_LIBRARIES})
endif()

if(OPENMP_FOUND)
   target_link_libraries(${PROJECTNAME} ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS})
endif()

if(SIFTGPU_FOUND)
   icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/elas/include
                                             EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${SIFTGPU_INCLUDE_DIRS}
                                             DESTINATION include/iCub/${PROJECTNAME}
                                             FILES ${folder_header})
else()
   icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/elas/include
                                             EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES}
                                             DESTINATION include/iCub/${PROJECTNAME}
                                             FILES ${folder_header})
endif()
