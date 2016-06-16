# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)

file(GLOB header ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB source ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${stereoVision_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

if(USE_SIFT_GPU)
   include_directories(${SIFTGPU_INCLUDE_DIRS})
   add_definitions(-DUSING_GPU)
endif()

add_definitions(-D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} ${source} ${header})
target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

if(USE_SIFT_GPU)
   target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

