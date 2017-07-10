# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Sean Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(stereo-vision)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH}
                              ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

set(USE_SIFT_GPU FALSE CACHE BOOL "Select false to run without GPU, or true for GPU acceleration")
if(USE_SIFT_GPU)
    find_package(SIFTGPU REQUIRED)
endif()

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES})
include(YarpInstallationHelpers)

## add libraries
add_subdirectory(lib)

## then modules
add_subdirectory(msgs)
add_subdirectory(modules)

## then apps
add_subdirectory(app)

icubcontrib_finalize_export(stereoVision)
icubcontrib_add_uninstall_target()
