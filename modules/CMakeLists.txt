# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
set(PROJECTNAME stereoModules)

project(${PROJECTNAME})

set(USE_SFM FALSE CACHE BOOL "Select false not to compile SFM")
if(USE_SFM)
    add_subdirectory(SFM)
endif()
set(USE_DISPARITYMODULE FALSE CACHE BOOL "Select false not to compile DisparityModule")
if(USE_DISPARITYMODULE)
    add_subdirectory(DisparityModule)
endif()
