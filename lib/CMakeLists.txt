# Copyright: (C) 2011 RobotCub Consortium
# Author: Sean Ryan Fanello, Giulia Pasquale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(stereoVision)

############## LIBELAS ##############

list(APPEND CMAKE_CXX_FLAGS "-msse3")

if(UNIX)
    find_package(OpenMP)
endif()

if(OPENMP_FOUND)
  message(STATUS "OpenMP FOUND")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

  set(folder_source_elas src/elas/elas_omp.cpp
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
else()
  message(STATUS "OpenMP NOT FOUND")
  set(folder_source_elas src/elas/elas.cpp
                         src/elas/descriptor.cpp
                         src/elas/filter.cpp
                         src/elas/matrix.cpp
                         src/elas/triangle.cpp)
endif()

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
                  include/iCub/stereoVision/elasWrapper.h
                  include/iCub/stereoVision/elas/elas.h
                  include/iCub/stereoVision/elas/descriptor.h
                  include/iCub/stereoVision/elas/image.h
                  include/iCub/stereoVision/elas/filter.h
                  include/iCub/stereoVision/elas/timer.h
                  include/iCub/stereoVision/elas/matrix.h
                  include/iCub/stereoVision/elas/triangle.h)

if(USE_SIFT_GPU)
    list(APPEND folder_source src/utils.cpp)
    list(APPEND folder_header include/iCub/stereoVision/utils.h)
endif()

if(OpenCV_VERSION_MAJOR GREATER 2)
  add_definitions(-DOPENCV_GREATER_2)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/iCub/stereoVision/elas)

source_group("Source Files" FILES ${folder_source})
source_group("ELAS Source Files" FILES ${folder_source_elas})
source_group("Header Files" FILES ${folder_header})

include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
    add_definitions(-DUSING_GPU)
endif()

add_library(${PROJECT_NAME} ${folder_source} ${folder_source_elas} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} iKin ${YARP_LIBRARIES})

if(USE_SIFT_GPU)
    target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

if(OPENMP_FOUND)
   target_link_libraries(${PROJECT_NAME} ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS})
endif()

if(USE_SIFT_GPU)
   icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/iCub/stereoVision/elas
                                              EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES} ${SIFTGPU_INCLUDE_DIRS}
                                              DESTINATION include/iCub/${PROJECT_NAME}
                                              FILES_WITH_PATH ${folder_header}
                                              PATH_TO_EXCLUDE include/iCub/stereoVision)
else()
   icubcontrib_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/iCub/stereoVision/elas
                                              EXTERNAL_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${OpenCV_LIBRARIES}
                                              DESTINATION include/iCub/${PROJECT_NAME}
                                              FILES_WITH_PATH ${folder_header}
                                              PATH_TO_EXCLUDE include/iCub/stereoVision)
endif()
