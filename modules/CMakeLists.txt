# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME stereoModules)

project(${PROJECTNAME})

set(USE_SFM FALSE CACHE BOOL "Select false not to compile SFM")
if(USE_SFM) 
    add_subdirectory(SFM)
endif()
set(USE_SCENEFLOW FALSE CACHE BOOL "Select false not to compile sceneFlow")
if(USE_SCENEFLOW)
    add_subdirectory(sceneFlow)
endif()
add_subdirectory(pointCloudConversion)
