cmake_minimum_required(VERSION 2.6)

project(stereoVision)

find_package(YARP)
find_package(ICUB)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

find_package(OpenCV)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

add_subdirectory(app)
add_subdirectory(src)
