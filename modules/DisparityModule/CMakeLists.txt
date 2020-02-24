# Copyright: 2019 IIT
# Author: Malafronte Damiano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(DisparityModule)

add_subdirectory(sgbm_cuda)

set(USE_DISPARITYMODULE_GUI ON CACHE BOOL "Select ON to compile with the GUI code")

set(source DispModule.cpp StereoMatcher.cpp)
set(header DispModule.h StereoMatcher.h)

if(USE_DISPARITYMODULE_GUI)
    list(APPEND source cvgui.cpp)
    list(APPEND header cvgui.h)
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories("../../lib/include")

include_directories(${stereoVision_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/sgbm_cuda")

add_executable(${PROJECT_NAME} ${source} ${header})

if(USE_SIFT_GPU)
    include_directories(${SIFTGPU_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE USING_GPU)
endif()

if(USE_DISPARITYMODULE_GUI)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_GUI)
endif()

target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} SGBM_CUDA ${CMAKE_DL_LIBS})

if(USE_SIFT_GPU)
    target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
