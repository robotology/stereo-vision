# Copyright: (C) Alberto Cardellino
# Authors: Alberto Cardellino (alberto.cardellino@iit.it)
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 3.5)

project(RGBD2pointCloud)


###########
## Build ##
###########

# headers
set( msg_include_dirs ${msgs_root_SOURCE_DIR})

# idl for thirft
yarp_add_idl(IDL_GEN_FILES idl.thrift)
source_group("IDL Files" FILES idl.thrift)

# include directories list
include_directories( ${msg_include_dirs} )

add_executable(${PROJECT_NAME} PointCloudConverter.cpp RGBD2PointCloud.cpp idl.thrift ${IDL_GEN_FILES})

target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

##########
## Link ##
##########

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} YARP::YARP_rosmsg)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} DESTINATION bin)


###########
## Tests ##
###########

