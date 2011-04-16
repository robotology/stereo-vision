include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(AUXFCN
	../auxiliaryFunctions/Cvtools.h
	../auxiliaryFunctions/Cvtools.cpp
	../auxiliaryFunctions/Camera.h
    ../auxiliaryFunctions/Camera.cpp
	../auxiliaryFunctions/stereoCamera.h
	../auxiliaryFunctions/stereoCamera.cpp)

add_subdirectory(stereoCalib)
add_subdirectory(stereoDisparity)