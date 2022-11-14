# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Sean Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(stereo-vision)

find_package(ICUBcontrib REQUIRED)
find_package(YARP 3.5.1 COMPONENTS os sig dev math cv rosmsg idl_tools REQUIRED)
find_package(ICUB 2.0.0 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH}
                              ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#set(USE_SIFT_GPU FALSE CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")
#if(USE_SIFT_GPU)
#    find_package(SIFTGPU REQUIRED)
#endif()

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

## locate OpenCV libraries
find_package(OpenCV REQUIRED)

set(COMPILE_LIB FALSE CACHE BOOL "Select false not to compile library")

## add libraries
if(COMPILE_LIB)
    add_subdirectory(lib)
endif()

## then modules
add_subdirectory(modules)

## then apps
add_subdirectory(app)

#icubcontrib_finalize_export(stereoVision)
icubcontrib_add_uninstall_target()
