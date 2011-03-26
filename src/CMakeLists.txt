include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/auxiliaryFunctions)

add_subdirectory(stereoCalib)
add_subdirectory(stereoDisparity)