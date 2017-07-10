# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME stereoModules)

project(${PROJECTNAME})

add_subdirectory(SFM)
add_subdirectory(sceneFlow)
add_subdirectory(pointCloudConversion)
