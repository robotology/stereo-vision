# Copyright: (C) Alberto Cardellino
# Authors: Alberto Cardellino (alberto.cardellino@iit.it)
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

project(RGBD2pointCloud)


###########
## Build ##
###########

# headers
set( msg_include_dirs ${msgs_root_SOURCE_DIR})


# include directories list
include_directories( ${YARP_INCLUDE_DIRS} ${msg_include_dirs} )

add_executable(${PROJECT_NAME} PointCloudConverter.cpp RGBD2PointCloud.cpp)

##########
## Link ##
##########

target_link_libraries( ${PROJECT_NAME} ${YARP_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} DESTINATION bin)


###########
## Tests ##
###########

